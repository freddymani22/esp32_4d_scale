FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY server.py .

# Create uploads directory
RUN mkdir -p uploads

# Flask port
EXPOSE 8080
# TCP trigger port
EXPOSE 9000

CMD ["python", "server.py"]
