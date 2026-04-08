# just a demo
FROM python:3.10

WORKDIR /tests

COPY ./requirements.txt .
RUN pip install -r requirements.txt

COPY tests/ ./tests

CMD ["python", "tests/test_example.py"]
