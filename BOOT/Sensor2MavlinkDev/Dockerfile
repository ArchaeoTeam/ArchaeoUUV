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
        "name": "Michael Bommhardt-Richter",\
        "email": "m.bommhardt@gmx.de"\
    }\
]'
LABEL docs ''
LABEL company '{\
        "about": "",\
        "name": "Archaeocopter",\
        "email": "Archaeocopter"\
    }'
LABEL readme 'http://www.archaeocopter.de/'
LABEL website 'http://www.archaeocopter.de/'
LABEL support 'http://www.archaeocopter.de/'
LABEL requirements="core >= 1"

ENTRYPOINT pigpiod && cd /app && GPIOZERO_PIN_FACTORY=pigpio python main.py