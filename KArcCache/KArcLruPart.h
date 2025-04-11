#pragma once

# include <unordered_map>
# include <mutex>
# include "KArcCacheNode.h"

namespace KamaCache
{

template<typename Key, typename Value>
class ArcLruPart
{
public:
    using NodeType = ArcNode<Key, Value>;
    using NodePtr = std::shared_ptr<NodeType>;
    using NodeMap = std::unordered_map<Key, NodePtr>;

    explicit ArcLruPart(size_t capacity, size_t transformThreshold)
        : capacity_(capacity)
        , ghostCapacity_(capacity)
        , transformThreshold_(transformThreshold)
    {
        initializeLists();
    }

    bool put(Key key, Value value)
    {
        if (capacity_ == 0) return false;

        std::lock_guard<std::mutex> lock(mutex_);
        auto it = mainCache_.find(key);
        if (it != mainCache_.end())
        {
            return updateExistingNode(it->second, value);
        }
        return addNewNode(key, value);
    }

    bool get(Key key, Value& value, bool& shouldTransform)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = mainCache_.find(key);
        if (it != mainCache_.end())
        {
            shouldTransform = updateNodeAccess(it->second);
            value = it->second->getValue();
            return true;
        }
        return false;
    }

    bool checkGhost(Key key)
    {
        auto it = ghostCache_.find(key);
        if (it != ghostCache_.end()) {
            removeFromGhost(it->second);
            ghostCache_.erase(it);
            return true;
        }
        return false;
    }

    void increaseCapacity() { ++capacity_; };

    bool decreaseCapacity() 
    {
        if (capacity_ <= 0) return false;
        if (mainCache_.size() == capacity_) {
            evictLeastRecent();
        }
        --capacity_;
        return true;
    }

private:
    void initializeLists()
    {
        mainHead_ = std::make_shared<NodeType>();
        mainTail_ = std::make_shared<NodeType>();
        mainHead_->next_ = mainTail_;
        mainTail_->prev_ = mainHead_;

        ghostHead_ = std::make_shared<NodeType>();
        ghostTail_ = std::make_shared<NodeType>();
        ghostHead_->next_ = ghostTail_;
        ghostTail_->prev_ = ghostHead_;
    }

    bool updateExistingNode(NodePtr node, const Value& value)
    {
        node->setValue(value);
        moveToFront(node);
        node->incrementAccessCount();
        return node->getAccessCount() >= transformThreshold_;
    }

    bool addNewNode(const Key& key, const Value& value)
    {
        if (mainCache_.size() >= capacity_)
        {
            evictLeastRecent(); // 驱逐最近最少访问
        }

        NodePtr newNode = std::make_shared<NodeType>(key, value);
        mainCache_[key] = newNode;
        addToFront(newNode);
        return true;

    }

    bool updateNodeAccess(NodePtr node)
    {
        moveToFront(node);
        node->incrementAccessCount();
        return node->getAccessCount() >= transformThreshold_;
    }

    bool moveToFront(NodePtr node)
    {
        node->prev_->next_ = node->next_;
        node->next_->prev_ = node->prev_;

        addToFront(node);
    }

    void addToFront(NodePtr node)
    {
        node->next_ = mainHead_->next_;
        node->prev_ = mainHead_;
        mainHead_->next_->prev_ = node;
        mainHead_->next_ = node;
    }

    void evictLeastRecent()
    {
        NodePtr leastRecent = mainTail_->prev_;
        if (leastRecent == mainHead_) 
            return;
        removeFromMain(leastRecent);
        if (ghostCache_.size() >= ghostCapacity_)
        {
            removeFromGhost();
        }
        addToGhost(leastRecent);
        mainCache_.erase(leastRecent->getKey());
    }

    void removeFromMain(NodePtr node)
    {
        node->prev_->next_ = node->next_;
        node->next_->prev_ = node->prev_;
    }

    void removeFromGhost(NodePtr node)
    {
        node->prev_->next_ = node->next_;
        node->next_->prev_ = node->prev_;
    }

    void addToGhost(NodePtr node)
    {
        node->accessCount_ = 1;
        node->next_ = ghostHead_->next_;
        node->prev_ = ghostHead_;
        ghostHead_->next->prev_ = node;
        ghostHead_->next = node;

        ghostCache_[node->getKey()] = node;
    }

    void removeOldestGhost()
    {
        NodePtr = oldestGhost = ghostTail_->prev_;
        if (oldestGhost == ghostHead_)
            return;
        removeFromGhost(oldestGhost);
        ghostCache_.erase(oldestGhost->getKey());
    }

private:
    size_t capacity_;
    size_t ghostCapacity_;
    size_t transformThreshold_; // 转换门槛值
    std::mutex mutex_;

    NodeMap mainCache_; // key -> ArcNode
    NodeMap ghostCache_;

    NodePtr mainHead_;
    NodePtr mainTail_;

    NodePtr ghostHead_;
    NodePtr ghostTail_;

};

}