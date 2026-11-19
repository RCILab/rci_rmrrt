#pragma once

#include <cstdlib>
#include <iostream>

#include <boost/program_options.hpp>

namespace rmrrt {

namespace po = boost::program_options;

// Parses named options. Prints the help and exits on -h/--help or on a bad option.
inline po::variables_map parseCommandLine(int argc, char** argv, po::options_description& options)
{
  options.add_options()("help,h", "show this help");
  po::variables_map values;
  try
  {
    po::store(po::parse_command_line(argc, argv, options), values);
    if (values.count("help"))
    {
      std::cout << options << "\n";
      std::exit(0);
    }
    po::notify(values);
  }
  catch (const po::error& error)
  {
    std::cerr << error.what() << "\n\n" << options << "\n";
    std::exit(2);
  }
  return values;
}

}  // namespace rmrrt
