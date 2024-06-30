const socket = io("http://localhost:5000/feed");

socket.on("connect", () => {
    console.log("Connected to server");

    // Join the feed for camera 0 (you can change this to other camera indices)
    socket.emit("join_feed", 0);
});

socket.on("camera_change", (cameras) => {
    console.log("Available cameras: ", cameras);
});

socket.on("send_feed_0", (data) => {
    const [buffer, startTime] = data;
    displayFrame(buffer, 0);
});

function displayFrame(buffer, cameraIndex) {
    const blob = new Blob([new Uint8Array(buffer)], { type: "image/jpeg" });
    const url = URL.createObjectURL(blob);

    let img = document.getElementById(`camera_${cameraIndex}`);
    if (!img) {
        img = document.createElement("img");
        img.id = `camera_${cameraIndex}`;
        document.getElementById("feeds").appendChild(img);
    }
    img.src = url;
}
