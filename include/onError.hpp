#pragma once
#include <functional>
#define LAMBDA(func) [](){func;}

struct errorFuncTuple{
   std::function<void()> func;
   float onError;
   bool called;
   errorFuncTuple(std::function<void()> func, float onError, bool called):
   func(func),onError(onError), called(called){}
};

