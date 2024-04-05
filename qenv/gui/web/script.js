function handleSpawn() {
    const spawn = document.getElementById("spawn_node"); 
    const destination = document.getElementById("destination_node");
    const remote = document.getElementById("remote");  

    if (remote.checked) {
        spawn.disabled = true; 
        destination.disabled = true; 
    } else {
        spawn.disabled = false; 
        destination.disabled = false; 
    }
}

function handleQCar() {
    const ip = document.getElementById("ip"); 
    const port = document.getElementById("port"); 
    const local = document.getElementById("local"); 

    if (local.checked) {
        ip.disabled = true; 
        port.disabled = true; 
    } else {
        ip.disabled = false; 
        port.disabled = false; 
    }
}

function handleDevice() {
    const device = document.getElementById("device"); 
    const keyboard = document.getElementById("keyboard"); 

    if (keyboard.checked) {
        device.disabled = true; 
    } else {
        device.disabled = false; 
    }
}

function handleImage() {
    const left = document.getElementById("left"); 
    const right = document.getElementById("right"); 
    const image = document.getElementById("spawn_image"); 

    if (left.checked) {
        image.src = "./images/Nodes_Right.PNG"; 
    } else if (right.checked) {
        image.src = "./images/Nodes_Left.PNG"; 
    }
}

function handleInit() {
    pywebview.api.load_json().then(function(response) {
        data = response.data; 
        console.log(data); 

        const controller = document.getElementById(data['controller'])
        controller.checked = true;
        controller.onchange(); 

        const operation = document.getElementById(data['operation_mode'])
        operation.checked = true; 
        operation.onchange(); 

        const traffic = document.getElementById(data['traffic'])
        traffic.checked = true; 
        traffic.onchange(); 

        const video = document.getElementById(data['video'])
        video.checked = true; 

        document.getElementById("csi_camera").checked = data["csi_camera"]; 
        document.getElementById("rgbd_camera").checked = data["rgbd_camera"]; 
        document.getElementById("lidar").checked = data["lidar"]; 
        
        document.getElementById("spawn_node").value = data["spawn_node"]; 
        document.getElementById("destination_node").value = data["destination_node"]; 
        document.getElementById("port").value = data["port"]; 
        document.getElementById("ip").value = data['ip']; 
        document.getElementById("device").value = data["device"]; 
    }); 
}

function handleFilter() {
    filter = {} 

    if (document.getElementById("port").disabled) {
        filter["port"] = ""; 
    }
    if (document.getElementById("ip").disabled) {
        filter["ip"] = ""; 
    }
    if (document.getElementById("device").disabled) {
        filter["device"] = ""; 
    }
    if (document.getElementById("spawn_node").disabled) {
        filter["spawn_node"] = ""; 
    }
    if (document.getElementById("destination_node").disabled) {
        filter["destination_node"] = ""; 
    }

    return filter; 
} 

function handleApply() {
    data = {} 

    data["csi_camera"] = document.getElementById("csi_camera").checked; 
    data["rgbd_camera"] = document.getElementById("rgbd_camera").checked; 
    data["lidar"] = document.getElementById("lidar").checked; 

    const operation = document.getElementsByName("operation");
    for (const radioButton of operation) {
        if (radioButton.checked) {
            data["operation_mode"] = radioButton.id; 
            break; 
        }
    }

    const controller = document.getElementsByName("controller"); 
    for (const radioButton of controller) {
        if (radioButton.checked) {
            data["controller"] = radioButton.id; 
            break; 
        }
    }

    const traffic = document.getElementsByName("traffic"); 
    for (const radioButton of traffic) {
        if (radioButton.checked) {
            data["traffic"] = radioButton.id; 
            break; 
        }
    }

    const video = document.getElementsByName("video"); 
    for (const radioButton of video) {
        if (radioButton.checked) {
            data["video"] = radioButton.id; 
            break; 
        }
    }

    data["spawn_node"] = document.getElementById("spawn_node").value; 
    data["destination_node"] = document.getElementById("destination_node").value; 
    data["port"] = document.getElementById("port").value; 
    data['ip'] = document.getElementById("ip").value; 
    data["device"] = document.getElementById("device").value

    return data; 
    
}

function handleSubmit() {
    data = handleApply(); 
    pywebview.api.apply_setting(data); 
    filter = handleFilter(); 
    pywebview.api.apply_filter(filter); 
}

window.onload = function() {
    setTimeout(handleInit, 500); 
}