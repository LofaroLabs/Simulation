#!/bin/bash

echoFunction() {
  echo "echo is Called"
}
fooBar() {
  echo "Functions are FUN!"
}

fooBar;
echoFunction;
# You call functions without (), just the function name then a semicolon.
