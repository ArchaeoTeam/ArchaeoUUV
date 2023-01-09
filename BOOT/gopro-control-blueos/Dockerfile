from python:3.9-slim-bullseye

COPY install.sh /install.sh

RUN /install.sh

COPY app /app
RUN python /app/setup.py install

EXPOSE 80/tcp

LABEL version="1.0.0"
# TODO: Add a Volume for persistence across boots
LABEL permissions '\
{\
  "ExposedPorts": {\
    "80/tcp": {}\
  },\
  "HostConfig": {\
    "Privileged": true,\
    "Binds":["/root/.config:/root/.config"],\
    "PortBindings": {\
      "80/tcp": [\
        {\
          "HostPort": ""\
        }\
      ]\
    }\
  }\
}'
LABEL authors '[\
    {\
        "name": "Thomas-David Griedel",\
        "email": "Griedel911@gmail.com"\
    }\
]'
LABEL docs ''
LABEL company '{\
        "about": "",\
        "name": "HTW Dresden",\
        "email": ""\
    }'
LABEL website 'https://github.com/davo911'
LABEL requirements="core >= 1"

ENTRYPOINT pigpiod && cd /app && GPIOZERO_PIN_FACTORY=pigpio python main.py