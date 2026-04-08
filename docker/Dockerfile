# just a demo
FROM python:3.10

WORKDIR /test

COPY test/requirements.txt .
RUN pip install -r requirements.txt

COPY test/ ./test

CMD ["python", "test/test_example.py"]
