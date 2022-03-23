## Playing around with code
## Basic tests with ThreadPoolExecutor
### Two executors
from concurrent.futures import ThreadPoolExecutor
import time

def fct_test_1():
    time.sleep(5)
    return 5

def fct_test_2():
    time.sleep(10)
    return 10

executor = ThreadPoolExecutor(max_workers=2)

# submit soes not interrupt the function
# a and b are Future objects
a = executor.submit(fct_test_1)
b = executor.submit(fct_test_2)

# result() function waits function to return

# In this scenario we get "5" after 5 seconds and "10" after 5 more seconds
# print(a.result())
# print(b.result())

# In this scenario we get "10" and "5" after 10 seconds
print(b.result())
print(a.result())