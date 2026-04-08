# Use official Python image
FROM python:3.10

# Set working directory inside container
WORKDIR /app

# Copy requirements first (better caching)
COPY requirements.txt .

# Install dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy source code
COPY src/ ./src

# Copy tests
COPY tests/ ./tests

# Default command (you can change this)
CMD ["python", "tests/test_example.py"]
