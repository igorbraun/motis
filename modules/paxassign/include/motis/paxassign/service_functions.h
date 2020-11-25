#pragma once

namespace motis::paxassign {

template <typename T>
std::size_t get_vec_size_bytes(std::vector<T> const& vec) {
  return sizeof(std::vector<T>) + (sizeof(T) * vec.size());
}

template <typename T>
double get_vec_of_vec_mb_size(std::vector<std::vector<T>> const& vec_of_vec) {
  if (vec_of_vec.size() == 0) return 0;
  return vec_of_vec.size() * get_vec_size_bytes(vec_of_vec[0]) / 1024 / 1024;
}

}  // namespace motis::paxassign