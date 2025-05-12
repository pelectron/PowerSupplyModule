#include "psm.hpp"

namespace psm {
HandleRef get_psm() {
  static constinit BasicHandle basic_handle{};
  return HandleRef(basic_handle);
}
} // namespace psm
