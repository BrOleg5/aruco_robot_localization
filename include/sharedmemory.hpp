/**
 * Wrappers is based on official Quick Guide for Impatient, in section Creating vectors in shared memory
 * @link https://www.boost.org/doc/libs/1_38_0/doc/html/interprocess/quick_guide.html
 */

#ifndef SHAREDMEMORY_HPP
#   define SHAREDMEMORY_HPP

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>

using namespace boost::interprocess;

namespace shm
{
    /**
     * @class Transmitter
     * @brief Boost shared memory wrapper.
     */
    template<class T>
    class Transmitter
    {
        private:

        const char *filename;
        managed_shared_memory segment;

        //Alias an STL compatible allocator of ints that allocates ints from the managed
        //shared memory segment.  This allocator will allow to place containers
        //in managed shared memory segments
        typedef allocator<T, managed_shared_memory::segment_manager> ShmemAllocator;

        //Alias a vector that uses the previous STL-like allocator
        typedef vector<T, ShmemAllocator> MyVector;

        public:

        MyVector *data;

        /**
         * Initialize shared memory.
         * 
         * @param file_name name of shared memory file.
         * @param file_size size of shared memory file.
         * @param data_size
         */
        Transmitter(const char *file_name, int file_size) : filename(file_name) {
            //First remove any old shared memory of the same name, create 
            //the shared memory segment and initialize needed resources
            shared_memory_object::remove(filename);
            segment = managed_shared_memory(create_only, filename, file_size);
            //Initialize shared memory STL-compatible allocator
            const ShmemAllocator alloc_inst (segment.get_segment_manager());

            //Construct a shared memory
            data = segment.construct<MyVector>("MyVector") (alloc_inst);
        }
        ~Transmitter() {
            segment.destroy<MyVector>("MyVector");
            shared_memory_object::remove(filename);
        }
    };

    /**
     * @class Receiver
     * @brief Boost shared memory wrapper.
     */
    template<class T>
    class Receiver
    {
        private:

        const char *filename;
        managed_shared_memory segment;
        //Alias an STL compatible allocator of ints that allocates ints from the managed
        //shared memory segment.  This allocator will allow to place containers
        //in managed shared memory segments
        typedef allocator<T, managed_shared_memory::segment_manager> ShmemAllocator;

        //Alias a vector that uses the previous STL-like allocator
        typedef vector<T, ShmemAllocator> MyVector;

        public:

        MyVector* data;

        /**
         * Initialize shared memory.
         * 
         * @param file_name name of shared memory file.
         * @param data_size element number of array type T.
         */
        Receiver(const char *file_name) : filename(file_name) {
            //Connect to the already created shared memory segment
            //and initialize needed resources
            segment = managed_shared_memory(open_only, file_name);
            //Find the array
            data = segment.find<MyVector>("MyVector").first;
        }
        ~Receiver() {
            // segment.destroy<MyVector>("MyVector");
            // shared_memory_object::remove(filename);
        }
    };
}

#endif