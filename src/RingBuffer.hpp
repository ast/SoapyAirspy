// Copyright 2024 SM6WJM

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdexcept>

#include <cstddef>
#include <cstdint>
#include <sys/mman.h>
#include <unistd.h>

// The Cortex-A7 in the RPi3 has a 64-byte cache line size (L2 cache)
#define cacheline_aligned alignas(64)

template <typename T> class RingBuffer {

  T *buffer_;
  const size_t capacity_;

  cacheline_aligned std::atomic<size_t> read_pos_{0};
  cacheline_aligned std::atomic<size_t> write_pos_{0};

  size_t read_pos_cached_ = 0;
  size_t write_pos_cached_ = 0;
  size_t available_cached_ = 0;
  size_t free_cached_ = 0;

  // We need these because there's not timed out wait on std::atomic.
  cacheline_aligned std::mutex lock_;
  cacheline_aligned std::condition_variable cond_;

  // Unmap mirror memory
  static void unmap_mirror(const void *addr, const size_t size) noexcept {
    int munmap_res;
    munmap_res = munmap(const_cast<void *>(addr), 2 * size);
    if (munmap_res == -1) {
      std::cerr << "munmap failed" << std::endl;
    }
  }

  static bool is_power_of_two(const size_t val) {
    return (val != 0) && ((val & (val - 1)) == 0);
  }

  // Double mapped memory for "magic" ring buffer.
  static T *map_mirror(const size_t size) {
    // Get page size
    const auto pagesize = static_cast<size_t>(getpagesize());
    if (size < pagesize) {
      throw std::runtime_error(
          "Capacity must be at least pagesize: " + std::to_string(pagesize) +
          " requested size: " + std::to_string(size));
    }

    if (not is_power_of_two(size)) {
      throw std::runtime_error(
          "Capacity must be a power of two, requested size: " +
          std::to_string(size));
    }

    // Create a memfd. Name is only for debugging purposes and can
    // be reused.
    int mem_fd = memfd_create("soapy_ring_buffer", 0);
    if (mem_fd == -1) {
      // TODO: add exact error
      throw std::runtime_error("Could not create memfd: " +
                               std::string(strerror(errno)));
    }

    // Truncate to size
    const int ftruncate_res = ftruncate(mem_fd, static_cast<__off_t>(size));
    if (ftruncate_res == -1) {
      throw std::runtime_error("Could not ftruncate memfd: " +
                               std::string(strerror(errno)));
    }

    // Find a piece of memory of size 2 * size.
    void *buffer =
        mmap(NULL, 2 * size, PROT_NONE, MAP_ANONYMOUS | MAP_PRIVATE, -1, 0);
    if (buffer == MAP_FAILED) {
      throw std::runtime_error("Could not mmap buffer: " +
                               std::string(strerror(errno)));
    }

    // Map the memfd to the first half of the buffer
    void *addr_hint_1 = buffer;
    void *buffer_1 = mmap(addr_hint_1, size, PROT_READ | PROT_WRITE,
                          MAP_SHARED | MAP_FIXED, mem_fd, 0);

    if (buffer_1 == MAP_FAILED or buffer_1 != addr_hint_1) {
      throw std::runtime_error("Could not mmap buffer1: " +
                               std::string(strerror(errno)));
    }

    // Map the memfd to the second half of the buffer
    // The double cast is because of pointer arithmetic
    void *addr_hint_2 =
        static_cast<void *>(static_cast<uint8_t *>(buffer_1) + size);
    const void *buffer_2 = mmap(addr_hint_2, size, PROT_READ | PROT_WRITE,
                                MAP_SHARED | MAP_FIXED, mem_fd, 0);

    // Check if the second mmap was successful and that address is correct.
    if (buffer_2 == MAP_FAILED or buffer_2 != addr_hint_2) {
      throw std::runtime_error("Could not mmap buffer2: " +
                               std::string(strerror(errno)));
    }

    // Probably not strictly necessary.
    std::memset(buffer, 0, size);

    // All correct, return a pointer to the buffer
    return static_cast<T *>(buffer);
  }

  // Mask read and write pointer
  inline size_t mask(const size_t val) const noexcept {
    return (capacity_ - 1) & val;
  }

public:
  // Size in bytes, is a power of two.
  size_t size() const noexcept { return capacity_ * sizeof(T); }

  // Capacity in elements
  inline size_t capacity() const noexcept { return capacity_; }

  // Indicate number of elements written. Must only be called from
  // producer.
  inline void produce(const size_t elements) noexcept {
    // intentional wrap around arithmetic
    free_cached_ -= elements;
    write_pos_cached_ += elements;
    write_pos_.fetch_add(elements, std::memory_order_release);
    cond_.notify_one();
  }

  // Indicate number of elements read. Must only be called from
  // consumer.
  inline void consume(const size_t elements) noexcept {
    // intentional wrap around arithmetic
    available_cached_ -= elements;
    read_pos_cached_ += elements;
    read_pos_.fetch_add(elements, std::memory_order_release);
    cond_.notify_one();
  }

  // Available elements to read
  inline size_t available(const size_t required = 0) noexcept {
    if (available_cached_ < required) {
      available_cached_ =
          write_pos_.load(std::memory_order_acquire) - read_pos_cached_;
    }
    return available_cached_;
  }

  // Available space to write
  inline size_t free_to_write(const size_t required = 0) noexcept {
    if (free_cached_ < required) {
      free_cached_ = capacity_ - (write_pos_cached_ -
                                  read_pos_.load(std::memory_order_acquire));
    }
    return free_cached_;
  }

  // Pointer to read location. Must only be called from consumer.
  inline const T *read_ptr() const noexcept {
    return buffer_ + mask(read_pos_cached_);
  }

  // Pointer to write location. Must only be called from producer.
  inline T *write_ptr() const noexcept {
    return buffer_ + mask(write_pos_cached_);
  }

  // Rest buffer
  void clear() noexcept {
    std::unique_lock<std::mutex> lock(lock_);
    available_cached_ = 0;
    free_cached_ = capacity_;
    read_pos_cached_ = 0;
    write_pos_cached_ = 0;
    read_pos_.store(0, std::memory_order_release);
    write_pos_.store(0, std::memory_order_release);

    // Wake up producer and consumer.
    cond_.notify_all();
  }

  ssize_t
  read_at_least(const size_t elements, const std::chrono::microseconds &timeout,
                const std::function<size_t(const T *begin, const size_t avail)>
                    callback) {

    auto avail = available(elements);

    if (avail >= elements) {
      // We have enough elements, no need to wait
      const auto consumed = callback(read_ptr(), avail);
      consume(consumed);

      return static_cast<ssize_t>(consumed);
    }

    // Else wait for more elements
    std::unique_lock<std::mutex> lock(lock_);
    // Wait for enough data to be available
    if (cond_.wait_for(lock, timeout, [&] {
          avail = available(elements);
          return avail >= elements;
        })) {
      // Ok, we have enough data
      const auto consumed = callback(read_ptr(), avail);
      consume(consumed);

      return static_cast<ssize_t>(consumed);
    } else {
      // We timed out
      return -1;
    }
  }

  ssize_t write_at_least(
      const size_t elements, const std::chrono::microseconds &timeout,
      const std::function<size_t(T *begin, const size_t free)> callback) {

    auto free = free_to_write(elements);

    if (free >= elements) {
      // Ok, we have enough space, not need to wait
      const auto produced = callback(write_ptr(), free);
      produce(produced);

      return static_cast<ssize_t>(produced);
    }

    // Else wait for enough space to be available
    std::unique_lock<std::mutex> lock(lock_);
    // Wait for enough data to be available
    if (cond_.wait_for(lock, timeout, [&] {
          free = free_to_write(elements);
          return free >= elements;
        })) {
      // Ok, we have enough data
      const auto produced = callback(write_ptr(), free);
      produce(produced);

      return static_cast<ssize_t>(produced);
    } else {
      // We timed out
      return -1;
    }
  }

  explicit RingBuffer(size_t capacity)
      : buffer_(map_mirror(capacity * sizeof(T))), capacity_(capacity) {}

  virtual ~RingBuffer() { unmap_mirror(buffer_, capacity_ * sizeof(T)); };
};
