#include <cstddef>
#include <cstdint>
#ifdef TESTING
    #include <iostream> 
#endif // TESTING

template<typename Data_Size, const size_t Buffer_Size>
Ring_Buffer<Data_Size, Buffer_Size>::Ring_Buffer() 
    : buffer{0}, size{Buffer_Size}, unread{0}, head{0}, tail{0}
{}

template<typename Data_Size, const size_t Buffer_Size>
Ring_Buffer<Data_Size, Buffer_Size>::Ring_Buffer(const Ring_Buffer& rhs) 
    : size{rhs.size}
{
    unread = rhs.unread;
    head = rhs.head;
    tail = rhs.tail;

    for (size_t i = 0; i < rhs.size; ++i)
    {
        buffer[i] = rhs.buffer[i];
    }
}

template<typename Data_Size, const size_t Buffer_Size>
const Ring_Buffer<Data_Size, Buffer_Size>& 
Ring_Buffer<Data_Size, Buffer_Size>::operator=(const Ring_Buffer& rhs)
{
    if (this != &rhs)
    {
        unread = rhs.unread;
        head = rhs.head;
        tail = rhs.tail;

        for (size_t i = 0; i < rhs.size; ++i)
        {
            buffer[i] = rhs.buffer[i];
        }
    }

    return *this;
}

template<typename Data_Size, const size_t Buffer_Size>
Ring_Buffer<Data_Size, Buffer_Size>::~Ring_Buffer()
{
    clear();
}

template<typename Data_Size, const size_t Buffer_Size>
Data_Size& 
Ring_Buffer<Data_Size, Buffer_Size>::operator[](const Data_Size& idx)
{
    return buffer[idx];
}

template<typename Data_Size, const size_t Buffer_Size>
const Data_Size& 
Ring_Buffer<Data_Size, Buffer_Size>::operator[](const Data_Size& idx) const
{
    return buffer[idx];
}

template<typename Data_Size, const size_t Buffer_Size>
void 
Ring_Buffer<Data_Size, Buffer_Size>::put(Data_Size data)
{
    if (!full())   // to prevent overwriting unread data
    {
        buffer[head] = data;

        if (head == (size - 1))
        {
            head = 0;   // essence of "circular" buffer - restarts at beginning
        }
        else
        {
            ++head;     
        }

        ++unread;   // difference between read and unread updated
    }
}

template<typename Data_Size, const size_t Buffer_Size>
Data_Size
Ring_Buffer<Data_Size, Buffer_Size>::get()
{
    if (!empty())
    {
        Data_Size element = buffer[tail];

        if (tail == (size - 1))
        {
            tail = 0;   // essence of "circular" buffer - restarts at beginning
        }
        else
        {
            ++tail;
        }

        --unread;

        return element;
    }

    return 0;
}

template<typename Data_Size, const size_t Buffer_Size>
void 
Ring_Buffer<Data_Size, Buffer_Size>::clear()
{
    for (auto& element : buffer)
    {
        element = 0;
    }
    
    head = 0;
    tail = 0;
    unread = 0;
}

template<typename Data_Size, const size_t Buffer_Size>
size_t
Ring_Buffer<Data_Size, Buffer_Size>::get_size() const
{
    return size;
}

template<typename Data_Size, const size_t Buffer_Size>
size_t
Ring_Buffer<Data_Size, Buffer_Size>::get_unread() const
{
    return unread;
}

template<typename Data_Size, const size_t Buffer_Size>
bool 
Ring_Buffer<Data_Size, Buffer_Size>::full() const
{
    return (unread == size);
}

template<typename Data_Size, const size_t Buffer_Size>
bool 
Ring_Buffer<Data_Size, Buffer_Size>::empty() const
{
    return (unread == 0);
}

#ifdef TESTING
template<typename Data_Size, const size_t Buffer_Size>
void 
Ring_Buffer<Data_Size, Buffer_Size>::print()
{
    for (auto& element: buffer)
    {
        std::cout << static_cast<int>(element) << " ";
    }

    std::cout << '\n';
}
#endif // TESTING
