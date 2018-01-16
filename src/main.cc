#include <mediator.hpp>
#include <iostream>

struct IoStreamsBase {
  // stream dedicated to the expression of errors
  std::ostream& err;

  // input stream
  std::istream& in;

  // output stream
  std::ostream& out;
};


int main() {
  holden::mediator::mediator m{};
  IoStreamsBase streams {
    .err = std::cerr,
    .in = std::cin,
    .out = std::cout,
  };



  return 0;
}