#pragma once

#include "../KICachePolicy.h"
#include "KArcLruPart.h"
#include "KArcLfuPart.h"
#include <memory>

namespace KamaCache
{

template<typename Key, typename Value>
class KArcCache : public KICachePolicy<Key, Value>
{

public:
    explicit KArcCache(size_t capacity = 10, size_t transformThreshold = 2)
        : capacity_(capacity)
        , transformThreshold_(transformThreshold)
        , lruPart_(std::make_unique<ArcLruPart<Key, Value>>(capacity, transformThreshold))
        , lfuPart_(std::make_unique<ArcLfuPart<Key, Value>>(capacity, transformThreshold))
    {}

    ~KArcCache() override = default;

    void put(Key key, Value value) override
    {
        bool inGhost = checkGhostCaches(key);

        if (!inGhost)
        {
            if (lruPart_->put(key, value))
            {
                lfuPart_->put(key, value);
            } 
        } else {
            lruPart_->put(key, value);
        }
    }

    bool get(Key key, Value& value) override
    {
        checkGhostCaches(key);
        bool shouldTransform = false;
    }

    Value get(Key key) override
    {

    }

private:
    size_t capacity_;
    size_t transformThreshold_;
    std::unique_ptr<ArcLruPart<Key, Value>> lruPart_;
    std::unique_ptr<ArcLfuPart<Key, Value>> lfuPart_;

};


} // namespace KamaCache