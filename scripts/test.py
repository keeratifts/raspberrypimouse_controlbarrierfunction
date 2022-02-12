#! /usr/bin/env python

from numpy import linalg as la
import numpy as np

arr1 = np.arange(1, 10).reshape(1, 9)
arr2 = np.arange(10, 19).reshape(1, 9)

arr = np.vstack((arr1, arr2))

print (arr)

a = np.zeros((3,20))
a[0, :] = 1
print (a)

a = np.array([[4],[5]]) 
print(a)


a[[0, 1]] = a[[1, 0]]
print(a)