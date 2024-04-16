FROM skynet/base

WORKDIR /
COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt
ENTRYPOINT ["/bin/bash", "-l", "-c"]
WORKDIR /ws

ENTRYPOINT ["tail", "-f", "/dev/null"]
