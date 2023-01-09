# Gpio Control Example

This uses python to serve a static HTML page using Vue and Vuetify to stream the On-Board GoPro camera to a web browser. It also enables the user to control the GoPro camera.

Enable qemu static support with a docker

```
docker buildx create --name multiarch --driver docker-container --use
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

Then build it:

`docker buildx build --platform linux/amd64,linux/arm/v7 . -t YOURDOCKERHUBUSER/YOURDOCKERHUBREPO:latest --output type=registry
`

Then pull it in blueos:


```
red-pill
sudo docker run -d --net=host --name=blueos-example4 --restart=unless-stopped YOURDOCKERHUBUSER/YOURDOCKERHUBREPO:latest
```