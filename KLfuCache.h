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

    
private:
    int capacity_; 
    int minFreq_;
    int maxAverageNum_; // 最大平均访问频次
    int curAverageNum_; // 当前平均访问频次
    int curTotalNum_; // 当前访问所有缓存次数和
    std::mutex mutex_;
    NodeMap nodeMap_;
    std::unordered_map<int, FreqList<Key, Value>*> freqToFreqList_;
};


} // namespace KamaCache