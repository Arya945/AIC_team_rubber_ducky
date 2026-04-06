# test_example.py

# 🔹 A simple function to test
def add(a, b):
    return a + b


def divide(a, b):
    if b == 0:
        raise ValueError("Cannot divide by zero")
    return a / b


# 🔹 Test 1: basic addition
def test_add():
    assert add(2, 3) == 5


# 🔹 Test 2: negative numbers
def test_add_negative():
    assert add(-1, -1) == -2


# 🔹 Test 3: division works
def test_divide():
    assert divide(10, 2) == 5


# 🔹 Test 4: division by zero raises error
def test_divide_by_zero():
    try:
        divide(5, 0)
        assert False  # should not reach here
    except ValueError:
        assert True
