#ifndef SEQUENTIAL_CONTAINER_H
#define SEQUENTIAL_CONTAINER_H

#include <algorithm>
#include <cstddef>
#include <initializer_list>

namespace s21 {
    template <typename T>
    class sequential_container {
    public:
        using value_type = T;
        using reference = T&;
        using const_reference = const T&;
        using size_type = size_t;

        virtual bool empty() = 0;
        virtual size_type size() = 0;

        virtual void push(const_reference value) = 0;
        virtual void pop() = 0;
        virtual void swap(sequential_container& other) = 0;
    };
}  // namespace s21

#endif  // SEQUENTIAL_CONTAINER_H
