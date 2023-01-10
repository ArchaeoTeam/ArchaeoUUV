# Gpio Control Example

This uses python to serve a static HTML page using Vue and Vuetify controlling a gpio using a python backend
to build:
 
Enable qemu static support with a docker

```
docker buildx create --name multiarch --driver docker-container --use
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

Then build it:

`docker buildx build --platform linux/amd64,linux/arm/v7 . -t  archaeo4d/blueos_extensions --output type=registry
`

Then pull it in blueos:


```
red-pill
sudo docker run -d -p 6968:80 --name=blueos_extensions --restart=unless-stopped archaeo4d/blueos_extensions

```