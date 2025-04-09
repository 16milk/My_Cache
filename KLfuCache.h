#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <cmath>
#include <unordered_map>
#include <vector>
#include "KICachePolicy.h"

namespace KamaCache
{

template<typename Key, typename Value> class KLfuCache;

template<typename Key, typename Value> 
class FreqList
{
private:
    struct Node
    {
        int freq;
        Key key;
        Value value;
        std::shared_ptr<Node> pre;
        std::shared_ptr<Node> next;

        Node() 
        : freq(1), pre(nullptr), next(nullptr) {}
        Node(Key key, Value value) 
        : freq(1), key(key), value(value), pre(nullptr), next(nullptr) {}
    };

    using NodePtr = std::shared_ptr<Node>;
    int freq_;  // 访问频率
    NodePtr head_;
    NodePtr tail_;

public:
    explicit FreqList(int n)
    : freq_(n)
    {
        head_ = std::make_shared<Node>();
        tail_ = std::make_shared<Node>();
        head_->next = tail_;
        tail_->pre = head_;
    }

    bool isEmpty() const
    {
        return head_->next == tail_;
    }

    // 在尾部插入新结点
    void addNode(NodePtr node)
    {
        if (!node || !head_ || !tail_)
            return;
        
        node->pre = tail->pre;
        node->next = tail_;
        tail_->pre->next = node;
        tail_->pre = node;
    }

    void removeNode(NodePtr node)
    {
        if (!node || !head_ || !tail_)
            return;
        if (!node->pre || !node->next)
            return;
        
        node->pre->next = node->next;
        node->next->pre = node->pre;
        node->pre = nullptr;
        node->next = nullptr;
    }

    NodePtr getFirstNode() const 
        return head_->next;

    friend class KLfuCache<Key, Value>
};

template <typename Key, Value value>
class KLfuCache : public KICachePolicy<Key, Value>
{
public:
    using Node = typename FreqList<Key, Value>::Node;
    using NodePtr = std::shared_ptr<Node>;
    using NodeMap = std::unordered_map<Key, NodePtr>;

    KLfuCache(int capacity, int maxAverageNum = 10)
        :capacity_(capacity), minFreq_(INT8_MAX), maxAverageNum_(maxAverageNum),
         curAverageNum_(0), curTotalNum_(0)
    {}

    ~KLfuCache() override = default;
    
    void put(Key key, Value value) override
    {
        if (capacity_ == 0)
            return;
        
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = nodeMap_.find(key);
        if (it != nodeMap_.end())
        {
            it->second->value = value;
            getInternal(it->second, value); 
            return;
        }
        putInternal(key, value);
    }

    bool get(Key key, Value& value) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = nodeMap_.find(key);
        if (it != nodeMap_.end())
        {
            getInternal(it->second, value);
            return true;
        }
        return false;
    }

    Value get(Key key) override
    {
        Value value{};
        get(key, value);
        return value;
    }

    void purge()
    {
        nodeMap_.clear();
        freqToFreqList_.clear();
    }

private:
    void getInternal(NodePtr node, Value& value); // 获取缓存
    void putInternal(Key key, Value value); // 添加缓存

    void kickOut(); // 移除缓存中过期的数据

    void removeFromFreqList(NodePtr node);
    void addToFreqList(NodePtr node);

    void addFreqNum(); // 增加平均频次
    void decreaseFreqNum(int num); //减少平均访问频率
    void handleOverMaxAverageNum(); // 平均频率超过上限
    void updateMinFreq();

    
private:
    int capacity_; 
    int minFreq_; // 最小的频次，用于淘汰
    int maxAverageNum_; // 最大平均访问频次
    int curAverageNum_; // 当前平均访问频次
    int curTotalNum_; // 当前访问所有缓存次数和
    std::mutex mutex_;
    NodeMap nodeMap_;
    // 访问频次到该频次链表的映射
    std::unordered_map<int, std::shared_ptr<FreqList<Key, Value>>> freqToFreqList_;
};

template<typename Key, typename Value>
void KLfuCache<Key, Value>::getInternal(NodePtr node, Value& value)
{
    value = node->value;
    removeFromFreqList(node);
    node->freq_++;
    addToFreqList(node);
    if (node->freq_ - 1 == minFreq_ && freqToFreqList_[node->freq - 1]->isEmpty())
        minFreq_++;

    addFreqNum();
}

template<typename Key, typename Value>
void KLfuCache<Key, Value>::putInternal(Key key, Value value)
{
    if (nodeMap_.size() == capacity_)
    {
        kickOut();
    }
    NodePtr node = std::make_shared<Node>(key, value);
    nodeMap_[key] = node;
    addToFreqList(node);
    addFreqNum(); // 增加平均频次
    minFreq_ = std::min(minFreq_, 1);
}

template<typename Key, typename Value>
void KLfuCache<Key, Value>::kickOut()
{
    NodePtr node = freqToFreqList_[minFreq_]->getFirstNode();
    removeFromFreqList(node);
    nodeMap_.erase(node->key);
    decreaseFreqNum(node->freq);
}

template<typename Key, typename Value>
void KLfuCache<Key, Value>::removeFromFreqList(NodePtr node)
{
    if (!node) return;
    size_t freq = node->freq;
    freqToFreqList_[freq]->removeNode(node);
}

template<typename Key, typename Value>
void KLfuCache<Key, Value>::addToFreqList(NodePtr node)
{
    if (!node) return;
    size_t freq = node->freq;
    if (freqToFreqList_.find(freq) == freqToFreqList_.end())
    {
        freqToFreqList_[freq] = make_shared<FreqList<Key, Value>>(freq);
    }
    freqToFreqList_[freq]->addNode(node);
}

template<typename Key, typename Value>
void KLfuCache<Key, Value>::addFreqNum()
{
    curTotalNum_++;
    if (nodeMap_.empty()) 
        curAverageNum_ = 0;
    else 
        curAverageNum_ = curTotalNum_ / nodeMap_.size();

    if (curAverageNum_ > maxAverageNum_)
    {
        handleOverMaxAverageNum();
    }
}

template<typename Key, typename Value>
void KLfuCache<Key, Value>::decreaseFreqNum(int num)
{
    curTotalNum_ -= num;
    if (nodeMap_.empty())
        curAverageNum_ = 0;
    else 
        curAverageNum_ = curTotalNum_ / nodeMap_.size();
}

template<typename Key, typename Value>
void KLfuCache<Key, Value>::handleOverMaxAverageNum()
{
    if (nodeMap_.empty()) 
        return;

    for (auto it = nodeMap_.begin(); it != nodeMap_.end(); ++it)
    {
        // 检查结点是否为空
        if (!it->second)
            continue;
        NodePtr node = it->second;
        removeFromFreqList(node);
        node->freq -= maxAverageNum / 2;
        if (node->freq < 1) node->freq = 1;
        addToFreqList(node);
    }
    updateMinFreq();
}

template<typename Key, typename Value>
void KLfuCache<Key, Value>::updateMinFreq()
{
    minFreq_ = INT8_MAX;
    for (const auto& pair : freqToFreqList_)
    {
        if (pair.second && !pair.second->isEmpty())
        {
            minFreq_ = std::min(minFreq_, pair.first);
        }
    }
    if (minFreq_ == INT8_MAX)
        minFreq_ = 1;
}

template<typename Key, typename Value>
class KHashLfuCache
{
public:
    KHashLfuCache(size_t capacity, int sliceNum, int maxAverageNum = 10)
        : sliceNum_(sliceNum > 0 ? sliceNum : std::thread::hardware_concurrency())
        , capacity_(capacity)
    {
        size_t sliceSize = std::ceil(capacity_ / static_cast<double>(sliceNum_));
        for (int i = 0; i < sliceNum_; ++i)
        {
            lfuSliceCaches_.emplace_back(make_shared<KLfuCache<Key, Value>>(sliceSize, maxAverageNum));
        }
    }

    void put(Key key, Value value)
    {
        size_t sliceIndex = Hash(key) % sliceNum_;
        return lfuSliceCaches_[sliceIndex]->put(key, value);
    }

    bool get(Key key, Value& value)
    {
        size_t sliceIndex = Hash(key) % sliceNum_;
        return lfuSliceCaches_[sliceIndex]->get(key, value);
    }

    Value get(Key key)
    {
        Value value{};
        get(key, value);
        return value;
    }

    void purge()
    {
        for (auto& lfuSliceCache : lfuSliceCaches_)
        {
            lfuSliceCache->purge();
        }
    }

private:
    size_t Hash(Key key)
    {
        std::hash<Key> hashFunc;
        return hashFunc(key);
    }

private:
    size_t capacity_;  // 缓存总容量
    int sliceNum_;   // 缓存分片数量
    std::vector<std::unique_ptr<KLfuCache<Key, Value>>> lfuSliceCaches_;

};

} // namespace KamaCache