file(REMOVE_RECURSE
  "../../devel/lib/libjack_framework_common.pdb"
  "../../devel/lib/libjack_framework_common.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/jack_framework_common.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
