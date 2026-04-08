# just a demo
FROM python:3.10

WORKDIR /app

COPY docker/requirements.txt .
RUN pip install -r requirements.txt

COPY app/ ./app

CMD ["python", "app/main.py"]
