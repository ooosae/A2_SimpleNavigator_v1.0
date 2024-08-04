#ifndef QUEUE_H
#define QUEUE_H

#include <initializer_list>
#include <stdexcept>

#include "s21_sequential_container.h"

namespace s21 {
    template <typename T>
    class queue : public sequential_container<T> {
    public:
        struct Node {
            T data;
            Node* next;
            explicit Node(const T& value) : data(value), next(nullptr) {}
        };

        using value_type = typename sequential_container<T>::value_type;
        using reference = typename sequential_container<T>::reference;
        using const_reference = typename sequential_container<T>::const_reference;
        using size_type = typename sequential_container<T>::size_type;

        queue() : frontNode(nullptr), backNode(nullptr) {}

        queue(std::initializer_list<value_type> const& items)
                : frontNode(nullptr), backNode(nullptr) {
            for (const value_type& item : items) {
                this->push(item);
            }
        }

        queue(const queue& q) : frontNode(nullptr), backNode(nullptr) {
            Node* current = q.frontNode;
            while (current != nullptr) {
                push(current->data);
                current = current->next;
            }
        }

        queue(queue&& q) noexcept : frontNode(q.frontNode), backNode(q.backNode) {
            q.frontNode = nullptr;
            q.backNode = nullptr;
        }

        ~queue() {
            while (frontNode != nullptr) {
                Node* temp = frontNode;
                frontNode = frontNode->next;
                delete temp;
            }
        }

        queue& operator=(queue&& q) noexcept {
            if (this != &q) {
                while (frontNode != nullptr) {
                    Node* temp = frontNode;
                    frontNode = frontNode->next;
                    delete temp;
                }
                frontNode = q.frontNode;
                backNode = q.backNode;
                q.frontNode = nullptr;
                q.backNode = nullptr;
            }
            return *this;
        }

        const_reference front() {
            if (frontNode != nullptr) {
                return frontNode->data;
            } else {
                throw std::out_of_range("Queue is empty");
            }
        }

        const_reference back() {
            if (backNode != nullptr) {
                return backNode->data;
            } else {
                throw std::out_of_range("Queue is empty");
            }
        }

        bool empty() override { return frontNode == nullptr; }

        size_type size() override {
            size_type count = 0;
            Node* current = frontNode;
            while (current != nullptr) {
                ++count;
                current = current->next;
            }
            return count;
        }

        void push(const_reference value) override {
            Node* newNode = new Node(value);
            if (backNode == nullptr) {
                frontNode = newNode;
                backNode = newNode;
            } else {
                backNode->next = newNode;
                backNode = newNode;
            }
        }

        void pop() override {
            if (frontNode != nullptr) {
                Node* temp = frontNode;
                frontNode = frontNode->next;
                delete temp;

                if (frontNode == nullptr) {
                    backNode = nullptr;
                }
            } else {
                throw std::out_of_range("Queue is empty");
            }
        }


        void swap(sequential_container<T>& other) override {
            auto* otherQueue = dynamic_cast<queue<T>*>(&other);
            if (otherQueue) {
                std::swap(frontNode, otherQueue->frontNode);
                std::swap(backNode, otherQueue->backNode);
            } else {
                throw std::invalid_argument(
                        "Cannot swap with a different container type");
            }
        }

        template <typename... Args>
        void insert_many_back(Args&&... args) {
            (..., push(std::forward<Args>(args)));
        }

        class iterator {
        public:
            using value_type = T;
            using reference = T&;
            using pointer = T*;
            using difference_type = std::ptrdiff_t;
            using iterator_category = std::forward_iterator_tag;

            explicit iterator(Node* node) : current(node) {}

            reference operator*() { return current->data; }

            iterator& operator++() {
                if (current) {
                    current = current->next;
                }
                return *this;
            }

            iterator operator++(int) {
                iterator temp = *this;
                ++(*this);
                return temp;
            }

            bool operator==(const iterator& other) const {
                return current == other.current;
            }

            bool operator!=(const iterator& other) const {
                return current != other.current;
            }

        private:
            Node* current;
        };

        iterator begin() { return iterator(frontNode); }

        iterator end() { return iterator(nullptr); }

    private:
        Node* frontNode;
        Node* backNode;
    };

}  // namespace s21

#endif  // QUEUE_H
