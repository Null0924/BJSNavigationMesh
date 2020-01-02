"use strict";

var Class = require("abitbol");
var BABYLON = require("babylonjs");

var NullsFunnel = Class.$extend({
    __init__: function __init__() {
        this.portals = {};
        this.startPosition = null;
        this.endPosition = null;
    },

    addPolygon: function addPolygon(polygonId, p1, p2) {
        if (p2 === undefined) p2 = p1;

        var midPoint = this.getPortalMidPoint(p1, p2);
        var leftMidPoint = this.getPortalMidPoint(p1, midPoint);
        var rightMidPoint = this.getPortalMidPoint(midPoint, p2);
        this.portals[polygonId] = [p1, leftMidPoint, midPoint, rightMidPoint, p2];
    },

    getPortalMidPoint: function getPortalMidPoint(p1, p2) {
        var midPoint = new BABYLON.Vector3(0, 0, 0);
        midPoint.addInPlace(p1);
        midPoint.addInPlace(p2);
        return midPoint.scaleInPlace(1 / 2);
    },

    stringPull: function stringPull() {

        var currentPosition = this.startPosition;
        var path = [];

        console.log(this.portals);

        for (var polygonId in this.portals) {

            var distance = 90000;
            var currentIndex = -1;
            for (var index = 0; index < this.portals[polygonId].length; index++) {
                if (BABYLON.Vector3.Distance(this.portals[polygonId][index], currentPosition) < distance) {
                    distance = BABYLON.Vector3.Distance(this.portals[polygonId][index], currentPosition);
                    currentIndex = index;
                }
            }
            console.log("trasversing polygon id", polygonId);
            path.push(this.portals[polygonId][currentIndex]);
            currentPosition = this.portals[polygonId][currentIndex].clone();
        }

        path.push(this.endPosition);
        return path;
    }
});

module.exports = NullsFunnel;