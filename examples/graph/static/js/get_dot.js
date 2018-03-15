function onLoad() {
    const xMLHttpRequestGetNodeClassGraph = new XMLHttpRequest();
    xMLHttpRequestGetNodeClassGraph.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            dot = JSON.parse(this.responseText).dot;
            console.log(dot);
            document.getElementById("classGraph").innerHTML = Viz(dot);
        }
    };
    xMLHttpRequestGetNodeClassGraph.open("GET", "http://" + window.location.hostname + ":" + window.location.port + "/getClassGraph", true);
    xMLHttpRequestGetNodeClassGraph.send();

    const xMLHttpRequestGetNodeGraph = new XMLHttpRequest();
    xMLHttpRequestGetNodeGraph.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            dot = JSON.parse(this.responseText).dot;
            console.log(dot);
            document.getElementById("graph").innerHTML = Viz(dot);
        }
    };
    xMLHttpRequestGetNodeGraph.open("GET", "http://" + window.location.hostname + ":" + window.location.port + "/getGraph", true);
    xMLHttpRequestGetNodeGraph.send();
}
