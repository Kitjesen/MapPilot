#include <lingtu/sdk.hpp>

#include <cstdlib>
#include <iostream>

int main(int argc, char** argv) {
  try {
    const char* key = std::getenv("LINGTU_API_KEY");
    lingtu::sdk::Client robot(argc > 1 ? argv[1] : "http://127.0.0.1:5050", key ? key : "");
    std::cout << robot.system().health() << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
