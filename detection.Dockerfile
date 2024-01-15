FROM clrnet:latest

COPY ./src/detection /opt/detection
WORKDIR /opt/detection

ENTRYPOINT ["python3", "main.py"]