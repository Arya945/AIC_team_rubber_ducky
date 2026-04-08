# just a demo
FROM python:3.10

WORKDIR /test

COPY ./requirements.txt .
RUN pip install -r requirements.txt

COPY tests/ ./test

CMD ["python", "test/test_example.py"]
