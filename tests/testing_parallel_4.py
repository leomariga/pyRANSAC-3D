## Playing around with code
## Basic tests with ThreadPoolExecutor
### three executors
from concurrent.futures import ThreadPoolExecutor
import time

def fct_test_1():
    time.sleep(5)
    return 5

def fct_test_2():
    time.sleep(10)
    return 10

# You can avoid having to call this method explicitly if you use the with statement, 
# which will shutdown the Executor (waiting as if Executor.shutdown() were called with wait set to True):
#
with ThreadPoolExecutor(max_workers=2) as executor:
    # submit does not interrupt the function even when max_worker < submits
    # a and b are Future objects
    # calling three submit with 2 max_workers
    a = executor.submit(fct_test_1)
    print('ex a')
    b = executor.submit(fct_test_1)
    print('ex b')
    c = executor.submit(fct_test_1)
    print('ex c')
    # result() function waits function to return

    # executor creates a line of functions when called submit
    # In this scenario we get "5" and "5" after 5 seconds. Then, after 5 more seconds, "5"
    print(a.result())
    print('result a')
    print(b.result())
    print('result b')
    print(c.result())
    print('result c')