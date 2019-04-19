function createLib (execlib) {
  'use strict';

  var lib = execlib.lib;
  var ret = {};

  require('./mathcreator')(lib, ret);

  return ret;
}

module.exports = createLib;
