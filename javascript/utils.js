var gamejs=require('gamejs');
var vectors = gamejs.utils.vectors;

var normaliseAngle=exports.normaliseAngle=function(angle){
    while(angle>359)angle-=360;
    while(angle<0)angle+=360;
    return angle;
};

var degrees=exports.degrees=function(radians) {
    /*
    convert radians to degrees 
    */
    var pi = Math.PI;
    return (radians)*(180/pi);
};

var radians=exports.radians=function(degrees) {
    /*
    convert degrees to radians
    */
    var pi = Math.PI;
    return (degrees)*(pi/180);
};


exports.rotateVector=function(vect, angle){
    /*
    vect - vector as array [x, y]
    angle - angle to rotate by, degrees
    
    returns rotated vector
    */
    if(angle>0){
        angle=radians(angle);
        return [vect[0]* Math.cos(angle)-vect[1]*Math.sin(angle),
                vect[0]* Math.sin(angle)+vect[1]*Math.cos(angle)];
    }else if(angle < 0){
        angle=radians(-angle);
        return [vect[0]*Math.cos(angle)+vect[1]*Math.sin(angle),
                -1*vect[0]*Math.sin(angle)+vect[1]*Math.cos(angle)];
    }
    else{
        return vect;
    }
};

exports.vectorDotProduct=function(vect1, vect2){
    /*
    vect1, vect2 - vectors as arrays [x, y],
    returns dot product - float
    */
    return vect1[0]*vect2[0]+vect1[1]*vect2[1];
};

var vectorLength=exports.vectorLength=function(vect){
    /*
    vect - vector as array. returns length
    returns float
    */
    return Math.sqrt(vect[0]*vect[0]+vect[1]*vect[1]);
};

var normaliseVector=exports.normaliseVector=function(vect){
    /*
     
    */
    var len=vectorLength(vect);
    if(len>0) return [vect[0]/len, vect[1]/len];
    return [0, 0];
};

exports.angleBetweenVectors=function(vect1, vect2){
    var len1=vectorLength(vect1);
    var len2=vectorLength(vect2);
    if(len1&&len2){
        var cosan=(vect1[0]*vect2[0]+vect1[1]*vect2[1])/(len1*len2);
    }else return 0;

    try{
        return degrees(Math.acos(cosan));
    }catch(e){
        console.log(e);
        return 180;
    }
};
