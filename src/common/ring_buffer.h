//
// @date    1 February 2025
// @author  Calum Thornhill
// @brief   Basic ring buffer implementation designed for embedded applications.
//
// @details A ring buffer, or circular buffer, is a "dynamic" way of using a 
//          static array, and is commonly used in embedded applications that 
//          require no dynamic allocation. The buffer stores data that will 
//          be read at a future time. The class tracks two indexes, one index 
//          that marks the start of data that has just been inserted, and an 
//          index that marks the point of data that has not been read yet. Data
//          cannot be inserted past unread data to avoid losing that unread data.  
//          When data is read, the index tracking it is moved accordingly. This
//          design works especially well with interrupt based applications, 
//          where the interrupt fills the buffer, and a process outside the
//          interupt in the main routine transfers it elsewhere.
//
//          The class is template-based to provide customization for the data
//          type and number of elements. It provides methods to view the size 
//          of the entire static array and the number of unread elements within
//          the array. 
//

#ifndef RING_BUFFER_HPP
#define RING_BUFFER_HPP

#include <cstdint>
#include <cstddef>

template<typename Data_Size, const size_t Buffer_Size>
class Ring_Buffer
{
public:
    Ring_Buffer();
    Ring_Buffer(const Ring_Buffer& rhs);
    const Ring_Buffer& operator=(const Ring_Buffer& rhs);
    ~Ring_Buffer();

    Data_Size& operator[](const Data_Size& idx);
    const Data_Size& operator[](const Data_Size& idx) const;

    void put(Data_Size data);
    Data_Size get();
 
    void clear();

    size_t get_size() const;        // array size
    size_t get_unread() const;      // elements in array
    bool full() const;
    bool empty() const;
#ifdef TESTING
    void print();
#endif

private:
    Data_Size buffer[Buffer_Size];
    const size_t size;      // size of static array 
    size_t unread;          // number of elements between head and tail
    Data_Size head;         // location of next element to be inserted 
    Data_Size tail;         // location of next element to be read
};

#include "ring_buffer.hpp"

#endif // RING_BUFFER_HPP
