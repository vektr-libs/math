function createMath (lib,mylib) {
  'use strict';

  function updateBBox(bbox,point){
    if(point[0]<bbox[0]){
      bbox[0]=point[0];
    }
    if(point[0]>bbox[1]){
      bbox[1]=point[0];
    }
    if(point[1]<bbox[2]){
      bbox[2]=point[1];
    }
    if(point[1]>bbox[3]){
      bbox[3]=point[1];
    }
  }

  function arcToSegments(bb, x, y, rx, ry, large, sweep, rotateX, ox, oy) {
    var th = rotateX * (Math.PI/180);
    var sin_th = Math.sin(th);
    var cos_th = Math.cos(th);
    rx = Math.abs(rx);
    ry = Math.abs(ry);
    var px = cos_th * (ox - x) * 0.5 + sin_th * (oy - y) * 0.5;
    var py = cos_th * (oy - y) * 0.5 - sin_th * (ox - x) * 0.5;
    var pl = (px*px) / (rx*rx) + (py*py) / (ry*ry);
    if (pl > 1) {
      pl = Math.sqrt(pl);
      rx *= pl;
      ry *= pl;
    }

    var a00 = cos_th / rx;
    var a01 = sin_th / rx;
    var a10 = (-sin_th) / ry;
    var a11 = (cos_th) / ry;
    var x0 = a00 * ox + a01 * oy;
    var y0 = a10 * ox + a11 * oy;
    var x1 = a00 * x + a01 * y;
    var y1 = a10 * x + a11 * y;

    var d = (x1-x0) * (x1-x0) + (y1-y0) * (y1-y0);
    var sfactor_sq = 1 / d - 0.25;
    if (sfactor_sq < 0) sfactor_sq = 0;
    var sfactor = Math.sqrt(sfactor_sq);
    if (sweep === large) sfactor = -sfactor;
    var xc = 0.5 * (x0 + x1) - sfactor * (y1-y0);
    var yc = 0.5 * (y0 + y1) + sfactor * (x1-x0);

    var th0 = Math.atan2(y0-yc, x0-xc);
    var th1 = Math.atan2(y1-yc, x1-xc);

    var th_arc = th1-th0;
    if (th_arc < 0 && sweep === 1){
      th_arc += 2*Math.PI;
    } else if (th_arc > 0 && sweep === 0) {
      th_arc -= 2 * Math.PI;
    }

    var segments = Math.ceil(Math.abs(th_arc / (Math.PI * 0.5 + 0.001)));
    var result = [], stb, rbb;
    for (var i=0; i<segments; i++) {
      var th2 = th0 + i * th_arc / segments;
      var th3 = th0 + (i+1) * th_arc / segments;
      stb = segmentToBezier(xc, yc, th2, th3, rx, ry, sin_th, cos_th);
      rbb = bezierBBox(x,y,stb[0],stb[1],stb[2],stb[3],stb[4],stb[5]);
      x = stb[4];
      y = stb[5];
      result[i] = stb;
    }

    return result;
  }


  function findDotAtSegment (p1x, p1y, c1x, c1y, c2x, c2y, p2x, p2y, t) {
      var t1 = 1 - t;
      return {
          x: Math.pow(t1, 3) * p1x + Math.pow(t1, 2) * 3 * t * c1x + t1 * 3 * t * t * c2x + Math.pow(t, 3) * p2x,
          y: Math.pow(t1, 3) * p1y + Math.pow(t1, 2) * 3 * t * c1y + t1 * 3 * t * t * c2y + Math.pow(t, 3) * p2y
      };
  }

  function bezierBBox(p1x, p1y, c1x, c1y, c2x, c2y, p2x, p2y) {
      var a = (c2x - 2 * c1x + p1x) - (p2x - 2 * c2x + c1x),
          b = 2 * (c1x - p1x) - 2 * (c2x - c1x),
          c = p1x - c1x,
          t1 = (-b + Math.sqrt(b * b - 4 * a * c)) / 2 / a,
          t2 = (-b - Math.sqrt(b * b - 4 * a * c)) / 2 / a,
          y = [p1y, p2y],
          x = [p1x, p2x],
          dot;
      if(Math.abs(t1) > "1e12"){
        t1 = 0.5;
      }
      if(Math.abs(t2) > "1e12"){
        t2 = 0.5;
      }
      if (t1 > 0 && t1 < 1) {
          dot = findDotAtSegment(p1x, p1y, c1x, c1y, c2x, c2y, p2x, p2y, t1);
          x.push(dot.x);
          y.push(dot.y);
      }
      if (t2 > 0 && t2 < 1) {
          dot = findDotAtSegment(p1x, p1y, c1x, c1y, c2x, c2y, p2x, p2y, t2);
          x.push(dot.x);
          y.push(dot.y);
      }
      a = (c2y - 2 * c1y + p1y) - (p2y - 2 * c2y + c1y);
      b = 2 * (c1y - p1y) - 2 * (c2y - c1y);
      c = p1y - c1y;
      t1 = (-b + Math.sqrt(b * b - 4 * a * c)) / 2 / a;
      t2 = (-b - Math.sqrt(b * b - 4 * a * c)) / 2 / a;
      if(Math.abs(t1) > "1e12"){
        t1 = 0.5;
      }
      if(Math.abs(t2) > "1e12"){
        t2 = 0.5;
      }
      if (t1 > 0 && t1 < 1) {
          dot = findDotAtSegment(p1x, p1y, c1x, c1y, c2x, c2y, p2x, p2y, t1);
          x.push(dot.x);
          y.push(dot.y);
      }
      if (t2 > 0 && t2 < 1) {
          dot = findDotAtSegment(p1x, p1y, c1x, c1y, c2x, c2y, p2x, p2y, t2);
          x.push(dot.x);
          y.push(dot.y);
      }
      return {min:[Math.min.apply(null,x), Math.min.apply(null,y)],
          max:[Math.max.apply(null,x), Math.max.apply(null,y)]};
  }

  function segmentToBezier(cx, cy, th0, th1, rx, ry, sin_th, cos_th) {
    var a00 = cos_th * rx;
    var a01 = -sin_th * ry;
    var a10 = sin_th * rx;
    var a11 = cos_th * ry;

    var th_half = 0.5 * (th1 - th0);
    var t = (8/3) * Math.sin(th_half * 0.5) * Math.sin(th_half * 0.5) / Math.sin(th_half);
    var x1 = cx + Math.cos(th0) - t * Math.sin(th0);
    var y1 = cy + Math.sin(th0) + t * Math.cos(th0);
    var x3 = cx + Math.cos(th1);
    var y3 = cy + Math.sin(th1);
    var x2 = x3 + t * Math.sin(th1);
    var y2 = y3 - t * Math.cos(th1);

    return [
      a00 * x1 + a01 * y1,      a10 * x1 + a11 * y1,
      a00 * x2 + a01 * y2,      a10 * x2 + a11 * y2,
      a00 * x3 + a01 * y3,      a10 * x3 + a11 * y3
    ];
  }

  function coordsToSpace(coords,matrix,resultcoords,start,length){ //coords be like [x1,y1,x2,y2,...,xN,yN]
    var st = start||0,index = st,x,y,len = length||coords.length;
    resultcoords = resultcoords || coords;
    while(index<st+len-1){ //stepping by 2
      x = matrix[0]*coords[index]+matrix[2]*coords[index+1]+matrix[4];
      y = matrix[1]*coords[index]+matrix[3]*coords[index+1]+matrix[5];
      resultcoords[index] = x;
      resultcoords[index+1] = y;
      index += 2;
    }
  }
  function coordsInSpace(coords,matrix){
    var ret = new Array(coords.length);
    coordsToSpace(coords,matrix,ret);
    return ret;
  }
  function pointToSpace(point,matrix){
    coordsToSpace(point,matrix);
  }
  function _pointToSpaceTranslator(point){
    pointToSpace(point,this);
  }
  function pointsToSpace(points,matrix){
    points.forEach (_pointToSpaceTranslator.bind(matrix));
  }
  function pointInSpace (point,matrix){
    return [
      matrix[0]*point[0]+matrix[2]*point[1]+matrix[4],
      matrix[1]*point[0]+matrix[3]*point[1]+matrix[5]
    ];
  }
  function lengthInSpace(len,matrix){
    var coords = [0,0,0,len],xl,yl;
    coordsToSpace(coords,matrix);
    xl = coords[2]-coords[0];
    yl = coords[3]-coords[1];
    return Math.sqrt(xl*xl+yl*yl);
  }
  function _pointInSpaceMaker(matrix,point,pointindex){
    this[pointindex] = pointInSpace(point,matrix);
  }
  function pointsInSpace(points,matrix){
    var ret = new Array(points.length);
    points.forEach(_pointInSpaceMaker.bind(ret,matrix));
    return ret;
  }
  function matrixInSpace(matrix, space) {
    return [
      matrix[0]*space[0]+matrix[2]*space[1],
      matrix[1]*space[0]+matrix[3]*space[1],
      matrix[0]*space[2]+matrix[2]*space[3],
      matrix[1]*space[2]+matrix[3]*space[3],
      matrix[0]*space[4]+matrix[2]*space[5]+matrix[4],
      matrix[1]*space[4]+matrix[3]*space[5]+matrix[5]
    ];
  }
  function matrixToSpace(matrix, space) {
    var a = matrix[0]*space[0]+matrix[2]*space[1],
      b = matrix[1]*space[0]+matrix[3]*space[1],
      c = matrix[0]*space[2]+matrix[2]*space[3],
      d = matrix[1]*space[2]+matrix[3]*space[3],
      e = matrix[0]*space[4]+matrix[2]*space[5]+matrix[4],
      f = matrix[1]*space[4]+matrix[3]*space[5]+matrix[5];
    matrix[0] = a;
    matrix[1] = b;
    matrix[2] = c;
    matrix[3] = d;
    matrix[4] = e;
    matrix[5] = f;
  }
  function inverseMatrix(m){
    var k = (m[0]*m[3]-m[1]*m[2]);
    return [m[3]/k,-m[1]/k,-m[2]/k,m[0]/k,(-m[3]*m[4]+m[2]*m[5])/k,(m[1]*m[4]-m[0]*m[5])/k];
  }
  function invertMatrixInPlace(m){
    var k = (m[0]*m[3]-m[1]*m[2]);
    var a = m[3]/k,
      b = -m[1]/k,
      c = -m[2]/k,
      d = m[0]/k,
      e = (-m[3]*m[4]+m[2]*m[5])/k,
      f = (m[1]*m[4]-m[0]*m[5])/k;
    m[0] = a;
    m[1] = b;
    m[2] = c;
    m[3] = d;
    m[4] = e;
    m[5] = f;
  }
  function boundingBoxToSpace(bb,m){
    var points = [[bb[0],bb[1]],[bb[0]+bb[2],bb[1]+bb[3]]];
    pointsToSpace(points,m);
    bb[0] = points[0][0];
    bb[1] = points[0][1];
    bb[2] = points[1][0]-bb[0];
    bb[3] = points[1][1]-bb[1];
  }
  function boundingBoxInSpace(bb,m){
    var points = bb.slice();
    boundingBoxToSpace(points,m);
    return points;
  }
  function boundingBoxesOverlap(bb1,bb2){
    if(bb2[0]>bb1[0]+bb1[2]){return false;}
    if(bb2[0]+bb2[2]<bb1[0]){return false;}
    if(bb2[1]>bb1[1]+bb1[3]){return false;}
    if(bb2[1]+bb2[3]<bb1[1]){return false;}
    return true;
  }
  function pointInBoundingBox (point, bb) {
    return point[0] >= bb[0] && point[0] <= bb[0] + bb[2] && point[1] >= bb[1] && point[1] <= bb[1] + bb[3];
  }
  mylib.updateBBox = updateBBox;
  mylib.arcToSegments = arcToSegments;
  mylib.bezierBBox = bezierBBox;
  mylib.segmentToBezier = segmentToBezier;
  mylib.coordsToSpace = coordsToSpace;
  mylib.coordsInSpace = coordsInSpace;
  mylib.pointToSpace = pointToSpace;
  mylib.pointsToSpace = pointsToSpace;
  mylib.pointInSpace = pointInSpace;
  mylib.lengthInSpace = lengthInSpace;
  mylib.pointsInSpace = pointsInSpace;
  mylib.matrixInSpace = matrixInSpace;
  mylib.matrixToSpace = matrixToSpace;
  mylib.inverseMatrix = inverseMatrix;
  mylib.invertMatrixInPlace = invertMatrixInPlace;
  mylib.boundingBoxToSpace = boundingBoxToSpace;
  mylib.boundingBoxInSpace = boundingBoxInSpace;
  mylib.boundingBoxesOverlap = boundingBoxesOverlap;
  mylib.pointInBoundingBox = pointInBoundingBox;
}

module.exports = createMath;
