/**
 * @file SIYI.cpp
 * @brief Explicit instantiation of the default SIYIDriver<64> ("SIYI")
 *        template. All driver logic lives in SIYI.h (SIYIDriver is a
 *        class template, so its definitions are necessarily header-based
 *        in standard C++ -- this is the same pattern used by e.g.
 *        std::vector). This translation unit exists so the library has a
 *        concrete .cpp per the requested layered file structure, and so
 *        the common SIYIDriver<64> instantiation is compiled exactly
 *        once instead of once per translation unit that includes SIYI.h.
 */
#include "SIYI.h"

namespace siyi {

// Explicit instantiation of the alias used throughout the examples/README.
template class SIYIDriver<64>;

} // namespace siyi
