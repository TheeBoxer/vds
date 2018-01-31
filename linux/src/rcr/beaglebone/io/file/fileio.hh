// Ok justin, we may want to implement more later, but let's only do the bare 
// minimum to not waste any time. Put this in the new file path here:
// https://github.com/nolanholden/vds/tree/master/linux/src/rcr/beaglebone/io/file

// the code below has not been compiled or tested--just getting you started.
// let me know of any questions you have!

#ifndef _RCR_BEAGLEBONE_IO_FILEIO_HH_
#define _RCR_BEAGLEBONE_IO_FILEIO_HH_

#include <mediator/mediator.hpp>

namespace rcr {
namespace beaglebone {
namespace io {
namespace file {

class FileIoHandler;

// WriteLine
struct FileWrite : holden::request<bool, FileIoHandler> {
  const char*& filepath;
  const char*& text;
};

class FileIoHandler
  : public holden::request_handler<Write> {
 public:
  auto handle(const FileWrite& m) -> Write::response_type {
    bool success = false;
    
    // if file at `filepath` doesn't exist, create it.
    // write the `text` to a file at `filepath`.
    
    // use only non-exception-throwing code. (probably means using
    // pure C.
    
    // return a bool indicating success or not (as below).
    
    return success;
  }
};

} // namespace file
} // namespace io
} // namespace beaglebone
} // namespace rcr

#endif // _RCR_BEAGLEBONE_IO_FILEIO_HH_
