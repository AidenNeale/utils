#!/bin/bash

# Blacklisted paths - add paths you want to exclude from linting
BLACKLISTED_PATHS=(
  "./build/*"
  "./.git/*"
)

# Build the exclusion arguments for find command
EXCLUDE_ARGS=""
for path in "${BLACKLISTED_PATHS[@]}"; do
  EXCLUDE_ARGS="$EXCLUDE_ARGS -not -path \"$path\""
done

# Run clang-tidy
eval "find . -type f \( -name \"*.cpp\" -o -name \"*.hpp\" \) $EXCLUDE_ARGS -exec clang-tidy -system-headers -p build {} +"

# Run clang-format
eval "find \".\" -regex '.*\.\(cpp\|hpp\|cc\|hh\|cxx\|hxx\|h\)' $EXCLUDE_ARGS -exec clang-format --dry-run -Werror -i {} +"