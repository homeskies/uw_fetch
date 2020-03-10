function convertToString(points) {
    // convert a list of points to string in the form of: p1X,p1Y p2X,p2Y p3X,p3Y
    var stringOfPoints = "";
    for (var i = 0; i < points.length; i++) {
        var point = points[i];
        stringOfPoints += point[0] + "," + point[1] + " ";
    }
    return stringOfPoints.trim();
}

function convertToList(stringOfPoints) {
    // convert a string of points in the form of: p1X,p1Y p2X,p2Y p3X,p3Y to a list of points
    var points = [];
    var stringSplitted = stringOfPoints.trim().split(" ");
    for (var i = 0; i < stringSplitted.length; i++) {
        var pointStr = stringSplitted[i].split(",");
        points.push([parseInt(pointStr[0]), parseInt(pointStr[1])]);
    }
    return JSON.parse(JSON.stringify(points));
}

function getRandomInteger(min, max) {
    return Math.floor(Math.random() * (max - min)) + min;
}

function getTranslate(translateStr) {
    // return the translate in the form of: [x, y]
    var translate = translateStr.substring(10, translateStr.length - 1);
    var translateArr = translate.split(",");
    return [parseInt(translateArr[0]), parseInt(translateArr[1])];
}

function getRegionId(selectedRegion) {
    // return the region id of the selected region
    var referencePointElement = selectedRegion.parentElement.childNodes[2];
	return parseInt(referencePointElement.getAttribute('id').split("-")[0]);
}

function getLabelElement(element) {
    // given a shape element, return the label of it
    return element.parentElement.childNodes[0];
}

function convertToDeg(radian) {
    // convert radian to degree
    return radian * (180 / Math.PI);
}

function round(num) {
    // roung the number to 2 decimal places
    return Math.round(num * 100) / 100;
}