/*
 * ATTENTION: The "eval" devtool has been used (maybe by default in mode: "development").
 * This devtool is neither made for production nor for readable output files.
 * It uses "eval()" calls to create a separate source file in the browser devtools.
 * If you are trying to read the output file, select a different devtool (https://webpack.js.org/configuration/devtool/)
 * or disable the default devtool with "devtool: false".
 * If you are looking for production-ready output files, see mode: "production" (https://webpack.js.org/configuration/mode/).
 */
/******/ (() => { // webpackBootstrap
/******/ 	"use strict";
/******/ 	var __webpack_modules__ = ({

/***/ "./node_modules/css-loader/dist/cjs.js!./src/style.css":
/*!*************************************************************!*\
  !*** ./node_modules/css-loader/dist/cjs.js!./src/style.css ***!
  \*************************************************************/
/***/ ((module, __webpack_exports__, __webpack_require__) => {

eval("__webpack_require__.r(__webpack_exports__);\n/* harmony export */ __webpack_require__.d(__webpack_exports__, {\n/* harmony export */   \"default\": () => (__WEBPACK_DEFAULT_EXPORT__)\n/* harmony export */ });\n/* harmony import */ var _node_modules_css_loader_dist_runtime_noSourceMaps_js__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ../node_modules/css-loader/dist/runtime/noSourceMaps.js */ \"./node_modules/css-loader/dist/runtime/noSourceMaps.js\");\n/* harmony import */ var _node_modules_css_loader_dist_runtime_noSourceMaps_js__WEBPACK_IMPORTED_MODULE_0___default = /*#__PURE__*/__webpack_require__.n(_node_modules_css_loader_dist_runtime_noSourceMaps_js__WEBPACK_IMPORTED_MODULE_0__);\n/* harmony import */ var _node_modules_css_loader_dist_runtime_api_js__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ../node_modules/css-loader/dist/runtime/api.js */ \"./node_modules/css-loader/dist/runtime/api.js\");\n/* harmony import */ var _node_modules_css_loader_dist_runtime_api_js__WEBPACK_IMPORTED_MODULE_1___default = /*#__PURE__*/__webpack_require__.n(_node_modules_css_loader_dist_runtime_api_js__WEBPACK_IMPORTED_MODULE_1__);\n// Imports\n\n\nvar ___CSS_LOADER_EXPORT___ = _node_modules_css_loader_dist_runtime_api_js__WEBPACK_IMPORTED_MODULE_1___default()((_node_modules_css_loader_dist_runtime_noSourceMaps_js__WEBPACK_IMPORTED_MODULE_0___default()));\n// Module\n___CSS_LOADER_EXPORT___.push([module.id, `/* style.css */\n\n* {\n  margin: 0;\n  padding: 0;\n  box-sizing: border-box;\n}\n\nbody, html {\n  height: 100%;\n  font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;\n  background: transparent;\n  overflow: hidden;\n  color: #ffffff;\n}\n\n.app-container {\n  display: flex;\n  height: 100vh;\n  overflow: hidden;\n}\n\n/* === Side Panel === */\n.side-panel {\n  width: 320px;\n  backdrop-filter: blur(10px);\n  background-color: rgb(84, 36, 36);\n  display: flex;\n  flex-direction: column;\n  padding: 20px;\n  border-right: 1px solid rgba(255, 255, 255, 0.1);\n}\n\n.panel-header {\n  text-align: center;\n  margin-bottom: 30px;\n}\n\n.logo {\n  width: 64px;\n  height: 64px;\n  margin-bottom: 10px;\n}\n\n.app-title {\n  font-size: 1.4rem;\n  color: #00d8ff;\n}\n\n/* === Control Panel & Info === */\n.control-panel, .info-panel {\n  margin-bottom: 30px;\n}\n\n.section-title {\n  font-size: 1.2rem;\n  margin-bottom: 10px;\n  color: #00d8ff;\n  display: flex;\n  align-items: center;\n  gap: 8px;\n}\n\n/* === Buttons === */\n.button-group {\n  display: flex;\n  flex-direction: column;\n  gap: 10px;\n}\n\n.control-button {\n  background-color: rgba(0, 216, 255, 0.2);\n  border: 1px solid #00d8ff;\n  color: #ffffff;\n  padding: 12px 16px;\n  font-size: 1rem;\n  border-radius: 8px;\n  display: flex;\n  align-items: center;\n  gap: 10px;\n  cursor: pointer;\n  transition: all 0.3s ease;\n}\n\n.control-button.primary {\n  background-color: rgba(0, 216, 255, 0.4);\n}\n\n.control-button:hover {\n  background-color: rgba(0, 216, 255, 0.6);\n}\n\n/* === Drone Info === */\n.status-grid {\n  display: grid;\n  grid-template-columns: 1fr;\n  gap: 12px;\n  margin-top: 10px;\n}\n\n.status-item {\n  display: flex;\n  align-items: center;\n  gap: 10px;\n  padding: 10px;\n  background-color: rgba(255, 255, 255, 0.05);\n  border-radius: 8px;\n  border: 1px solid rgba(255, 255, 255, 0.1);\n}\n\n.status-label {\n  flex: 1;\n  font-weight: 500;\n  color: #ffffff;\n}\n\n.status-value {\n  font-weight: bold;\n  color: #00ffb3;\n}\n\n/* === Cesium Container === */\n.main-content {\n  flex: 1;\n  position: relative;\n}\n\n#cesiumContainer {\n  width: 100%;\n  height: 100%;\n  position: absolute;\n  top: 0;\n  left: 0;\n  z-index: 0;\n}\n\n/* === Responsive Design === */\n@media (max-width: 768px) {\n  .app-container {\n    flex-direction: column;\n  }\n\n  .side-panel {\n    width: 100%;\n    height: auto;\n    border-right: none;\n    border-bottom: 1px solid rgba(255, 255, 255, 0.1);\n  }\n\n  .main-content {\n    height: calc(100% - 300px);\n  }\n}\n\n\n.cesium-timeline-main{\n    display: none;\n}\n`, \"\"]);\n// Exports\n/* harmony default export */ const __WEBPACK_DEFAULT_EXPORT__ = (___CSS_LOADER_EXPORT___);\n\n\n//# sourceURL=webpack:///./src/style.css?./node_modules/css-loader/dist/cjs.js");

/***/ }),

/***/ "./node_modules/css-loader/dist/runtime/api.js":
/*!*****************************************************!*\
  !*** ./node_modules/css-loader/dist/runtime/api.js ***!
  \*****************************************************/
/***/ ((module) => {

eval("\n\n/*\n  MIT License http://www.opensource.org/licenses/mit-license.php\n  Author Tobias Koppers @sokra\n*/\nmodule.exports = function (cssWithMappingToString) {\n  var list = [];\n\n  // return the list of modules as css string\n  list.toString = function toString() {\n    return this.map(function (item) {\n      var content = \"\";\n      var needLayer = typeof item[5] !== \"undefined\";\n      if (item[4]) {\n        content += \"@supports (\".concat(item[4], \") {\");\n      }\n      if (item[2]) {\n        content += \"@media \".concat(item[2], \" {\");\n      }\n      if (needLayer) {\n        content += \"@layer\".concat(item[5].length > 0 ? \" \".concat(item[5]) : \"\", \" {\");\n      }\n      content += cssWithMappingToString(item);\n      if (needLayer) {\n        content += \"}\";\n      }\n      if (item[2]) {\n        content += \"}\";\n      }\n      if (item[4]) {\n        content += \"}\";\n      }\n      return content;\n    }).join(\"\");\n  };\n\n  // import a list of modules into the list\n  list.i = function i(modules, media, dedupe, supports, layer) {\n    if (typeof modules === \"string\") {\n      modules = [[null, modules, undefined]];\n    }\n    var alreadyImportedModules = {};\n    if (dedupe) {\n      for (var k = 0; k < this.length; k++) {\n        var id = this[k][0];\n        if (id != null) {\n          alreadyImportedModules[id] = true;\n        }\n      }\n    }\n    for (var _k = 0; _k < modules.length; _k++) {\n      var item = [].concat(modules[_k]);\n      if (dedupe && alreadyImportedModules[item[0]]) {\n        continue;\n      }\n      if (typeof layer !== \"undefined\") {\n        if (typeof item[5] === \"undefined\") {\n          item[5] = layer;\n        } else {\n          item[1] = \"@layer\".concat(item[5].length > 0 ? \" \".concat(item[5]) : \"\", \" {\").concat(item[1], \"}\");\n          item[5] = layer;\n        }\n      }\n      if (media) {\n        if (!item[2]) {\n          item[2] = media;\n        } else {\n          item[1] = \"@media \".concat(item[2], \" {\").concat(item[1], \"}\");\n          item[2] = media;\n        }\n      }\n      if (supports) {\n        if (!item[4]) {\n          item[4] = \"\".concat(supports);\n        } else {\n          item[1] = \"@supports (\".concat(item[4], \") {\").concat(item[1], \"}\");\n          item[4] = supports;\n        }\n      }\n      list.push(item);\n    }\n  };\n  return list;\n};\n\n//# sourceURL=webpack:///./node_modules/css-loader/dist/runtime/api.js?");

/***/ }),

/***/ "./node_modules/css-loader/dist/runtime/noSourceMaps.js":
/*!**************************************************************!*\
  !*** ./node_modules/css-loader/dist/runtime/noSourceMaps.js ***!
  \**************************************************************/
/***/ ((module) => {

eval("\n\nmodule.exports = function (i) {\n  return i[1];\n};\n\n//# sourceURL=webpack:///./node_modules/css-loader/dist/runtime/noSourceMaps.js?");

/***/ }),

/***/ "./src/style.css":
/*!***********************!*\
  !*** ./src/style.css ***!
  \***********************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

eval("__webpack_require__.r(__webpack_exports__);\n/* harmony export */ __webpack_require__.d(__webpack_exports__, {\n/* harmony export */   \"default\": () => (__WEBPACK_DEFAULT_EXPORT__)\n/* harmony export */ });\n/* harmony import */ var _node_modules_style_loader_dist_runtime_injectStylesIntoStyleTag_js__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! !../node_modules/style-loader/dist/runtime/injectStylesIntoStyleTag.js */ \"./node_modules/style-loader/dist/runtime/injectStylesIntoStyleTag.js\");\n/* harmony import */ var _node_modules_style_loader_dist_runtime_injectStylesIntoStyleTag_js__WEBPACK_IMPORTED_MODULE_0___default = /*#__PURE__*/__webpack_require__.n(_node_modules_style_loader_dist_runtime_injectStylesIntoStyleTag_js__WEBPACK_IMPORTED_MODULE_0__);\n/* harmony import */ var _node_modules_style_loader_dist_runtime_styleDomAPI_js__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! !../node_modules/style-loader/dist/runtime/styleDomAPI.js */ \"./node_modules/style-loader/dist/runtime/styleDomAPI.js\");\n/* harmony import */ var _node_modules_style_loader_dist_runtime_styleDomAPI_js__WEBPACK_IMPORTED_MODULE_1___default = /*#__PURE__*/__webpack_require__.n(_node_modules_style_loader_dist_runtime_styleDomAPI_js__WEBPACK_IMPORTED_MODULE_1__);\n/* harmony import */ var _node_modules_style_loader_dist_runtime_insertBySelector_js__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! !../node_modules/style-loader/dist/runtime/insertBySelector.js */ \"./node_modules/style-loader/dist/runtime/insertBySelector.js\");\n/* harmony import */ var _node_modules_style_loader_dist_runtime_insertBySelector_js__WEBPACK_IMPORTED_MODULE_2___default = /*#__PURE__*/__webpack_require__.n(_node_modules_style_loader_dist_runtime_insertBySelector_js__WEBPACK_IMPORTED_MODULE_2__);\n/* harmony import */ var _node_modules_style_loader_dist_runtime_setAttributesWithoutAttributes_js__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! !../node_modules/style-loader/dist/runtime/setAttributesWithoutAttributes.js */ \"./node_modules/style-loader/dist/runtime/setAttributesWithoutAttributes.js\");\n/* harmony import */ var _node_modules_style_loader_dist_runtime_setAttributesWithoutAttributes_js__WEBPACK_IMPORTED_MODULE_3___default = /*#__PURE__*/__webpack_require__.n(_node_modules_style_loader_dist_runtime_setAttributesWithoutAttributes_js__WEBPACK_IMPORTED_MODULE_3__);\n/* harmony import */ var _node_modules_style_loader_dist_runtime_insertStyleElement_js__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! !../node_modules/style-loader/dist/runtime/insertStyleElement.js */ \"./node_modules/style-loader/dist/runtime/insertStyleElement.js\");\n/* harmony import */ var _node_modules_style_loader_dist_runtime_insertStyleElement_js__WEBPACK_IMPORTED_MODULE_4___default = /*#__PURE__*/__webpack_require__.n(_node_modules_style_loader_dist_runtime_insertStyleElement_js__WEBPACK_IMPORTED_MODULE_4__);\n/* harmony import */ var _node_modules_style_loader_dist_runtime_styleTagTransform_js__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! !../node_modules/style-loader/dist/runtime/styleTagTransform.js */ \"./node_modules/style-loader/dist/runtime/styleTagTransform.js\");\n/* harmony import */ var _node_modules_style_loader_dist_runtime_styleTagTransform_js__WEBPACK_IMPORTED_MODULE_5___default = /*#__PURE__*/__webpack_require__.n(_node_modules_style_loader_dist_runtime_styleTagTransform_js__WEBPACK_IMPORTED_MODULE_5__);\n/* harmony import */ var _node_modules_css_loader_dist_cjs_js_style_css__WEBPACK_IMPORTED_MODULE_6__ = __webpack_require__(/*! !!../node_modules/css-loader/dist/cjs.js!./style.css */ \"./node_modules/css-loader/dist/cjs.js!./src/style.css\");\n\n      \n      \n      \n      \n      \n      \n      \n      \n      \n\nvar options = {};\n\noptions.styleTagTransform = (_node_modules_style_loader_dist_runtime_styleTagTransform_js__WEBPACK_IMPORTED_MODULE_5___default());\noptions.setAttributes = (_node_modules_style_loader_dist_runtime_setAttributesWithoutAttributes_js__WEBPACK_IMPORTED_MODULE_3___default());\noptions.insert = _node_modules_style_loader_dist_runtime_insertBySelector_js__WEBPACK_IMPORTED_MODULE_2___default().bind(null, \"head\");\noptions.domAPI = (_node_modules_style_loader_dist_runtime_styleDomAPI_js__WEBPACK_IMPORTED_MODULE_1___default());\noptions.insertStyleElement = (_node_modules_style_loader_dist_runtime_insertStyleElement_js__WEBPACK_IMPORTED_MODULE_4___default());\n\nvar update = _node_modules_style_loader_dist_runtime_injectStylesIntoStyleTag_js__WEBPACK_IMPORTED_MODULE_0___default()(_node_modules_css_loader_dist_cjs_js_style_css__WEBPACK_IMPORTED_MODULE_6__[\"default\"], options);\n\n\n\n\n       /* harmony default export */ const __WEBPACK_DEFAULT_EXPORT__ = (_node_modules_css_loader_dist_cjs_js_style_css__WEBPACK_IMPORTED_MODULE_6__[\"default\"] && _node_modules_css_loader_dist_cjs_js_style_css__WEBPACK_IMPORTED_MODULE_6__[\"default\"].locals ? _node_modules_css_loader_dist_cjs_js_style_css__WEBPACK_IMPORTED_MODULE_6__[\"default\"].locals : undefined);\n\n\n//# sourceURL=webpack:///./src/style.css?");

/***/ }),

/***/ "./node_modules/style-loader/dist/runtime/injectStylesIntoStyleTag.js":
/*!****************************************************************************!*\
  !*** ./node_modules/style-loader/dist/runtime/injectStylesIntoStyleTag.js ***!
  \****************************************************************************/
/***/ ((module) => {

eval("\n\nvar stylesInDOM = [];\nfunction getIndexByIdentifier(identifier) {\n  var result = -1;\n  for (var i = 0; i < stylesInDOM.length; i++) {\n    if (stylesInDOM[i].identifier === identifier) {\n      result = i;\n      break;\n    }\n  }\n  return result;\n}\nfunction modulesToDom(list, options) {\n  var idCountMap = {};\n  var identifiers = [];\n  for (var i = 0; i < list.length; i++) {\n    var item = list[i];\n    var id = options.base ? item[0] + options.base : item[0];\n    var count = idCountMap[id] || 0;\n    var identifier = \"\".concat(id, \" \").concat(count);\n    idCountMap[id] = count + 1;\n    var indexByIdentifier = getIndexByIdentifier(identifier);\n    var obj = {\n      css: item[1],\n      media: item[2],\n      sourceMap: item[3],\n      supports: item[4],\n      layer: item[5]\n    };\n    if (indexByIdentifier !== -1) {\n      stylesInDOM[indexByIdentifier].references++;\n      stylesInDOM[indexByIdentifier].updater(obj);\n    } else {\n      var updater = addElementStyle(obj, options);\n      options.byIndex = i;\n      stylesInDOM.splice(i, 0, {\n        identifier: identifier,\n        updater: updater,\n        references: 1\n      });\n    }\n    identifiers.push(identifier);\n  }\n  return identifiers;\n}\nfunction addElementStyle(obj, options) {\n  var api = options.domAPI(options);\n  api.update(obj);\n  var updater = function updater(newObj) {\n    if (newObj) {\n      if (newObj.css === obj.css && newObj.media === obj.media && newObj.sourceMap === obj.sourceMap && newObj.supports === obj.supports && newObj.layer === obj.layer) {\n        return;\n      }\n      api.update(obj = newObj);\n    } else {\n      api.remove();\n    }\n  };\n  return updater;\n}\nmodule.exports = function (list, options) {\n  options = options || {};\n  list = list || [];\n  var lastIdentifiers = modulesToDom(list, options);\n  return function update(newList) {\n    newList = newList || [];\n    for (var i = 0; i < lastIdentifiers.length; i++) {\n      var identifier = lastIdentifiers[i];\n      var index = getIndexByIdentifier(identifier);\n      stylesInDOM[index].references--;\n    }\n    var newLastIdentifiers = modulesToDom(newList, options);\n    for (var _i = 0; _i < lastIdentifiers.length; _i++) {\n      var _identifier = lastIdentifiers[_i];\n      var _index = getIndexByIdentifier(_identifier);\n      if (stylesInDOM[_index].references === 0) {\n        stylesInDOM[_index].updater();\n        stylesInDOM.splice(_index, 1);\n      }\n    }\n    lastIdentifiers = newLastIdentifiers;\n  };\n};\n\n//# sourceURL=webpack:///./node_modules/style-loader/dist/runtime/injectStylesIntoStyleTag.js?");

/***/ }),

/***/ "./node_modules/style-loader/dist/runtime/insertBySelector.js":
/*!********************************************************************!*\
  !*** ./node_modules/style-loader/dist/runtime/insertBySelector.js ***!
  \********************************************************************/
/***/ ((module) => {

eval("\n\nvar memo = {};\n\n/* istanbul ignore next  */\nfunction getTarget(target) {\n  if (typeof memo[target] === \"undefined\") {\n    var styleTarget = document.querySelector(target);\n\n    // Special case to return head of iframe instead of iframe itself\n    if (window.HTMLIFrameElement && styleTarget instanceof window.HTMLIFrameElement) {\n      try {\n        // This will throw an exception if access to iframe is blocked\n        // due to cross-origin restrictions\n        styleTarget = styleTarget.contentDocument.head;\n      } catch (e) {\n        // istanbul ignore next\n        styleTarget = null;\n      }\n    }\n    memo[target] = styleTarget;\n  }\n  return memo[target];\n}\n\n/* istanbul ignore next  */\nfunction insertBySelector(insert, style) {\n  var target = getTarget(insert);\n  if (!target) {\n    throw new Error(\"Couldn't find a style target. This probably means that the value for the 'insert' parameter is invalid.\");\n  }\n  target.appendChild(style);\n}\nmodule.exports = insertBySelector;\n\n//# sourceURL=webpack:///./node_modules/style-loader/dist/runtime/insertBySelector.js?");

/***/ }),

/***/ "./node_modules/style-loader/dist/runtime/insertStyleElement.js":
/*!**********************************************************************!*\
  !*** ./node_modules/style-loader/dist/runtime/insertStyleElement.js ***!
  \**********************************************************************/
/***/ ((module) => {

eval("\n\n/* istanbul ignore next  */\nfunction insertStyleElement(options) {\n  var element = document.createElement(\"style\");\n  options.setAttributes(element, options.attributes);\n  options.insert(element, options.options);\n  return element;\n}\nmodule.exports = insertStyleElement;\n\n//# sourceURL=webpack:///./node_modules/style-loader/dist/runtime/insertStyleElement.js?");

/***/ }),

/***/ "./node_modules/style-loader/dist/runtime/setAttributesWithoutAttributes.js":
/*!**********************************************************************************!*\
  !*** ./node_modules/style-loader/dist/runtime/setAttributesWithoutAttributes.js ***!
  \**********************************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

eval("\n\n/* istanbul ignore next  */\nfunction setAttributesWithoutAttributes(styleElement) {\n  var nonce =  true ? __webpack_require__.nc : 0;\n  if (nonce) {\n    styleElement.setAttribute(\"nonce\", nonce);\n  }\n}\nmodule.exports = setAttributesWithoutAttributes;\n\n//# sourceURL=webpack:///./node_modules/style-loader/dist/runtime/setAttributesWithoutAttributes.js?");

/***/ }),

/***/ "./node_modules/style-loader/dist/runtime/styleDomAPI.js":
/*!***************************************************************!*\
  !*** ./node_modules/style-loader/dist/runtime/styleDomAPI.js ***!
  \***************************************************************/
/***/ ((module) => {

eval("\n\n/* istanbul ignore next  */\nfunction apply(styleElement, options, obj) {\n  var css = \"\";\n  if (obj.supports) {\n    css += \"@supports (\".concat(obj.supports, \") {\");\n  }\n  if (obj.media) {\n    css += \"@media \".concat(obj.media, \" {\");\n  }\n  var needLayer = typeof obj.layer !== \"undefined\";\n  if (needLayer) {\n    css += \"@layer\".concat(obj.layer.length > 0 ? \" \".concat(obj.layer) : \"\", \" {\");\n  }\n  css += obj.css;\n  if (needLayer) {\n    css += \"}\";\n  }\n  if (obj.media) {\n    css += \"}\";\n  }\n  if (obj.supports) {\n    css += \"}\";\n  }\n  var sourceMap = obj.sourceMap;\n  if (sourceMap && typeof btoa !== \"undefined\") {\n    css += \"\\n/*# sourceMappingURL=data:application/json;base64,\".concat(btoa(unescape(encodeURIComponent(JSON.stringify(sourceMap)))), \" */\");\n  }\n\n  // For old IE\n  /* istanbul ignore if  */\n  options.styleTagTransform(css, styleElement, options.options);\n}\nfunction removeStyleElement(styleElement) {\n  // istanbul ignore if\n  if (styleElement.parentNode === null) {\n    return false;\n  }\n  styleElement.parentNode.removeChild(styleElement);\n}\n\n/* istanbul ignore next  */\nfunction domAPI(options) {\n  if (typeof document === \"undefined\") {\n    return {\n      update: function update() {},\n      remove: function remove() {}\n    };\n  }\n  var styleElement = options.insertStyleElement(options);\n  return {\n    update: function update(obj) {\n      apply(styleElement, options, obj);\n    },\n    remove: function remove() {\n      removeStyleElement(styleElement);\n    }\n  };\n}\nmodule.exports = domAPI;\n\n//# sourceURL=webpack:///./node_modules/style-loader/dist/runtime/styleDomAPI.js?");

/***/ }),

/***/ "./node_modules/style-loader/dist/runtime/styleTagTransform.js":
/*!*********************************************************************!*\
  !*** ./node_modules/style-loader/dist/runtime/styleTagTransform.js ***!
  \*********************************************************************/
/***/ ((module) => {

eval("\n\n/* istanbul ignore next  */\nfunction styleTagTransform(css, styleElement) {\n  if (styleElement.styleSheet) {\n    styleElement.styleSheet.cssText = css;\n  } else {\n    while (styleElement.firstChild) {\n      styleElement.removeChild(styleElement.firstChild);\n    }\n    styleElement.appendChild(document.createTextNode(css));\n  }\n}\nmodule.exports = styleTagTransform;\n\n//# sourceURL=webpack:///./node_modules/style-loader/dist/runtime/styleTagTransform.js?");

/***/ }),

/***/ "./src/config.js":
/*!***********************!*\
  !*** ./src/config.js ***!
  \***********************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

eval("__webpack_require__.r(__webpack_exports__);\n/* harmony export */ __webpack_require__.d(__webpack_exports__, {\n/* harmony export */   CESIUM_ION_TOKEN: () => (/* binding */ CESIUM_ION_TOKEN)\n/* harmony export */ });\nconst CESIUM_ION_TOKEN = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiI1MmI2MjkzYS1mMmE5LTRkODEtYTZiMC00YTZiMDgzZWU1YjAiLCJpZCI6ODYyNTEsImlhdCI6MTc0NjMxMjgxM30.OK5gINXOFgBwFNachWVbWWwYDpFeJXxNMS2_Ot6nMRg';\n\n//# sourceURL=webpack:///./src/config.js?");

/***/ }),

/***/ "./src/main.js":
/*!*********************!*\
  !*** ./src/main.js ***!
  \*********************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

eval("__webpack_require__.r(__webpack_exports__);\n/* harmony import */ var _config_js__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./config.js */ \"./src/config.js\");\n/* harmony import */ var _ui_controller_js__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./ui-controller.js */ \"./src/ui-controller.js\");\n/* harmony import */ var _style_css__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./style.css */ \"./src/style.css\");\n\n\nCesium.Ion.defaultAccessToken = _config_js__WEBPACK_IMPORTED_MODULE_0__.CESIUM_ION_TOKEN;\n\n\n\nwindow.viewer = new Cesium.Viewer(\"cesiumContainer\", {\n    shouldAnimate: true,\n    infoBox: false,\n    terrain: Cesium.Terrain.fromWorldTerrain(),\n    selectionIndicator: false,\n    shadows: true,\n    baseLayerPicker: false,\n    navigationHelpButton: false,\n    animation: false,\n    timeline: true,\n    fullscreenButton: false,\n    geocoder: false,\n    homeButton: false,\n    sceneModePicker: false,\n    projectionPicker: false,\n});\n\nconst viewer = window.viewer;\n\nviewer.cesiumWidget.creditContainer.style.display = \"none\";\nviewer.scene.fog.enabled = false;\nviewer.scene.globe.enableLighting = true;\nviewer.scene.globe.depthTestAgainstTerrain = true;\n\nconst mainJsAnimationConfig = {\n    start: Cesium.JulianDate.fromDate(new Date(2025, 2, 25, 16)),\n    duration: 40,\n    get stop() { return Cesium.JulianDate.addSeconds(this.start, this.duration, new Cesium.JulianDate()); }\n};\n\nviewer.clock.startTime = mainJsAnimationConfig.start.clone();\nviewer.clock.stopTime = mainJsAnimationConfig.stop.clone();\nviewer.clock.currentTime = mainJsAnimationConfig.start.clone();\nviewer.clock.multiplier = 2.0;\nviewer.clock.clockRange = Cesium.ClockRange.CLAMPED;\nviewer.clock.shouldAnimate = false;\n\nviewer.timeline.zoomTo(mainJsAnimationConfig.start, mainJsAnimationConfig.stop);\n\n\nlet points = [];\nfetch('/points.json', {\n    headers: {\n        'Accept': 'application/json'\n    }\n})\n    .then(response => {\n        if (!response.ok) {\n        }\n        return response.json();\n    })\n    .then(data => {\n        if (!data.dronePoints || !Array.isArray(data.dronePoints)) {\n        }\n\n        points = data.dronePoints.map((point, index) => {\n            if (!point.longitude || !point.latitude || !point.height) {\n            }\n            const cartesian = Cesium.Cartesian3.fromDegrees(point.longitude, point.latitude, point.height);\n            return cartesian;\n        });\n\n        if (points.length < 2) {\n        }\n\n        initializeDronePath();\n    })\n    .catch(error => {\n    });\n\nfunction initializeDronePath() {\n    if (!points || points.length < 2) {\n        return;\n    }\n    const sampledPosition = new Cesium.SampledPositionProperty();\n    const numberOfSamples = points.length;\n\n    for (let i = 0; i < numberOfSamples; i++) {\n        const time = Cesium.JulianDate.addSeconds(\n            mainJsAnimationConfig.start,\n            (i * mainJsAnimationConfig.duration) / (numberOfSamples - 1),\n            new Cesium.JulianDate()\n        );\n        sampledPosition.addSample(time, points[i]);\n    }\n\n    sampledPosition.setInterpolationOptions({\n        interpolationDegree: 3,\n        interpolationAlgorithm: Cesium.LinearApproximation\n    });\n\n    const orientation = new Cesium.VelocityOrientationProperty(sampledPosition);\n\n    for (let i = 0; i < points.length; ++i) {\n        viewer.entities.add({\n            position: points[i],\n        });\n    }\n\n    const entity = viewer.entities.add({\n        availability: new Cesium.TimeIntervalCollection([\n            new Cesium.TimeInterval({\n                start: mainJsAnimationConfig.start,\n                stop: mainJsAnimationConfig.stop,\n            }),\n        ]),\n        position: sampledPosition,\n        velocity: new Cesium.VelocityVectorProperty(sampledPosition),\n\n        orientation: orientation,\n        model: {\n            uri: \"./CesiumDrone.glb\",\n            minimumPixelSize: 256,\n            maximumScale: 10,\n            scale: 2.0,\n        },\n        path: {\n            show: false,\n            material: new Cesium.PolylineGlowMaterialProperty({\n                glowPower: 0.1,\n                color: Cesium.Color.RED,\n            }),\n            width: 10,\n            resolution: 0.01,\n            leadTime: 1,\n            trailTime: 0.1,\n        },\n\n        viewFrom: new Cesium.Cartesian3(-10, 5, 5),\n    });\n    if (!window.droneUI) {\n        window.droneUI = new _ui_controller_js__WEBPACK_IMPORTED_MODULE_1__[\"default\"]();\n    }\n    viewer.scene.camera.flyTo({\n        destination: Cesium.Cartesian3.fromDegrees(40.2925729751587, 40.61789421298509, 1500),\n        complete: function () {\n            setTimeout(function () {\n                viewer.scene.camera.flyTo({\n                    destination: Cesium.Cartesian3.fromDegrees(40.2925729751587, 40.61789421298509, 1200),\n                    duration: 3.0,\n                    complete: function () {\n                        viewer.trackedEntity = entity;\n                        window.droneUI.updateConnectionStatus('Bağlantı Kuruldu');\n\n                    }\n                });\n            }, 1000);\n        }\n    });\n\n\n\n\n document.getElementById('startButton').addEventListener('click', function () {\n        window.droneUI.startUpdates();\n        window.droneUI.updateConnectionStatus('Aktif');\n        entity.path.show = true;\n        entity.viewFrom = new Cesium.Cartesian3(-10, 0, -10);\n        viewer.trackedEntity = undefined;\n        viewer.trackedEntity = entity;\n        viewer.clock.startTime = mainJsAnimationConfig.start.clone();\n        viewer.clock.stopTime = mainJsAnimationConfig.stop.clone();\n        viewer.clock.currentTime = mainJsAnimationConfig.start.clone();\n        viewer.clock.multiplier = 1.0;\n        viewer.clock.clockRange = Cesium.ClockRange.CLAMPED;\n        viewer.clock.shouldAnimate = true;\n\n        // Linear interpolation (lerp) fonksiyonu\n        function lerp(start, end, t) {\n            return start * (1 - t) + end * t;\n        }\n\n        // Tek bir onTick listener kullanıyoruz\n        viewer.clock.onTick.addEventListener(function (clock) {\n            const currentTime = Cesium.JulianDate.secondsDifference(clock.currentTime, mainJsAnimationConfig.start);\n            const totalDuration = mainJsAnimationConfig.duration;\n            const transitionDuration = totalDuration * 0.1;\n\n            let currentSpeed;\n            if (currentTime < totalDuration * 0.3) {\n                if (currentTime < transitionDuration) {\n                    currentSpeed = lerp(0, 4, currentTime / transitionDuration);\n                } else {\n                    currentSpeed = 4;\n                }\n                viewer.clock.multiplier = 1.0;\n            }\n            else if (currentTime < totalDuration * 0.7) {\n                const accelerationTime = (currentTime - totalDuration * 0.3) / transitionDuration;\n                if (accelerationTime < 1) {\n                    currentSpeed = lerp(4, 12, accelerationTime);\n                } else {\n                    currentSpeed = 12;\n                }\n                viewer.clock.multiplier = 3.0;\n            }\n            else if (currentTime < totalDuration) {\n                const decelerationTime = (currentTime - totalDuration * 0.7) / transitionDuration;\n                if (currentTime > totalDuration - transitionDuration) {\n                    const finalSlowdown = (currentTime - (totalDuration - transitionDuration)) / transitionDuration;\n                    currentSpeed = lerp(4, 0, finalSlowdown);\n                } else if (decelerationTime < 1) {\n                    currentSpeed = lerp(12, 4, decelerationTime);\n                } else {\n                    currentSpeed = 4;\n                }\n                viewer.clock.multiplier = 1.0;\n            }\n            else {\n                currentSpeed = 0;\n                viewer.clock.shouldAnimate = false;\n                window.droneUI.updateConnectionStatus('Bağlantı Kesildi');\n                window.droneUI.updateAltitude(0);\n                window.droneUI.updateBatteryStatus(100);\n                window.droneUI.stopUpdates();\n            }\n\n            window.droneUI.updateSpeed(Math.round(currentSpeed));\n        });\n\n        entity.position.setInterpolationOptions({\n            interpolationDegree: 5,\n            interpolationAlgorithm: Cesium.LagrangePolynomialApproximation,\n        });\n        entity.trackingReferenceFrame = Cesium.TrackingReferenceFrame.VELOCITY;\n    });\n\n\n\n\n}\n\n//# sourceURL=webpack:///./src/main.js?");

/***/ }),

/***/ "./src/ui-controller.js":
/*!******************************!*\
  !*** ./src/ui-controller.js ***!
  \******************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

eval("__webpack_require__.r(__webpack_exports__);\n/* harmony export */ __webpack_require__.d(__webpack_exports__, {\n/* harmony export */   \"default\": () => (__WEBPACK_DEFAULT_EXPORT__)\n/* harmony export */ });\nclass DroneUIController {\n    constructor() {\n        this.connectionStatus = document.getElementById('connectionStatus');\n        this.batteryStatus = document.getElementById('batteryStatus');\n        this.altitudeStatus = document.getElementById('altitudeStatus');\n        this.speedStatus = document.getElementById('speedStatus');\n\n        this.startTime = null;\n        this.durationInterval = null;\n        this.batteryInterval = null;\n        this.cesiumTickListener = null;\n\n        this.initializeControls();\n    }\n\n    updateConnectionStatus(status) {\n        if (this.connectionStatus) {\n            this.connectionStatus.textContent = status;\n        }\n    }\n\n    updateBatteryStatus(percentage) {\n        if (this.batteryStatus) {\n            this.batteryStatus.textContent = `${percentage}%`;\n        }\n    }\n\n    updateAltitude(height) {\n        if (this.altitudeStatus) {\n            this.altitudeStatus.textContent = `${Math.round(height)}m`;\n        }\n    }\n\n    updateSpeed(speed) {\n        if (this.speedStatus) {\n            this.speedStatus.textContent = `${Math.round(speed)} km/h`;\n        }\n    }\n\n    initializeControls() {\n        if (this.connectionStatus) this.updateConnectionStatus('Çevrim Dışı');\n        if (this.batteryStatus) this.updateBatteryStatus(100);\n        if (this.altitudeStatus) this.updateAltitude(0);\n        if (this.speedStatus) this.updateSpeed(0);\n    }\n\n    stopUpdates() {\n        \n        if (this.batteryInterval) {\n            clearInterval(this.batteryInterval);\n            this.batteryInterval = null;\n        }\n        if (this.cesiumTickListener && window.viewer && window.viewer.clock) {\n            window.viewer.clock.onTick.removeEventListener(this.cesiumTickListener);\n            this.cesiumTickListener = null;\n        }\n        if (this.durationInterval) {\n            clearInterval(this.durationInterval);\n            this.durationInterval = null;\n        }\n    }\n\n    startUpdates() {\n        this.stopUpdates();\n\n        let currentBattery = 100;\n        this.updateBatteryStatus(currentBattery);\n        this.batteryInterval = setInterval(() => {\n            currentBattery = Math.max(0, currentBattery - 0.1);\n            this.updateBatteryStatus(Math.round(currentBattery));\n            if (currentBattery === 0) {\n                console.log(\"Batarya bitti!\");\n                this.updateConnectionStatus('Batarya Bitti!');\n                this.stopUpdates();\n            }\n        }, 1000);\n\n        this.cesiumTickListener = (clock) => {\n            const entity = window.viewer.trackedEntity;\n            if (entity && entity.position) {\n                const currentTime = clock.currentTime;\n                const position = entity.position.getValue(currentTime);\n\n                if (position) {\n                    const cartographic = Cesium.Cartographic.fromCartesian(position);\n                    const height = cartographic.height;\n                    this.updateAltitude(height-1150);\n\n                    let speed = 0;\n                    try {\n                        if (entity.velocity) {\n                            const velocityVector = entity.velocity.getValue(currentTime);\n                            if (velocityVector) {\n                                speed = Cesium.Cartesian3.magnitude(velocityVector);\n                            }\n                        }\n                    } catch (e) {\n                        console.warn('Hız hesaplanırken hata:', e);\n                    }\n                    this.updateSpeed(speed * 3.6);\n                }\n            }\n        };\n\n        if (window.viewer && window.viewer.clock) {\n            window.viewer.clock.onTick.addEventListener(this.cesiumTickListener);\n        } else {\n            console.error('Viewer veya clock bulunamadı!');\n        }\n    }\n}\n\n/* harmony default export */ const __WEBPACK_DEFAULT_EXPORT__ = (DroneUIController);\n\n//# sourceURL=webpack:///./src/ui-controller.js?");

/***/ })

/******/ 	});
/************************************************************************/
/******/ 	// The module cache
/******/ 	var __webpack_module_cache__ = {};
/******/ 	
/******/ 	// The require function
/******/ 	function __webpack_require__(moduleId) {
/******/ 		// Check if module is in cache
/******/ 		var cachedModule = __webpack_module_cache__[moduleId];
/******/ 		if (cachedModule !== undefined) {
/******/ 			return cachedModule.exports;
/******/ 		}
/******/ 		// Create a new module (and put it into the cache)
/******/ 		var module = __webpack_module_cache__[moduleId] = {
/******/ 			id: moduleId,
/******/ 			// no module.loaded needed
/******/ 			exports: {}
/******/ 		};
/******/ 	
/******/ 		// Execute the module function
/******/ 		__webpack_modules__[moduleId](module, module.exports, __webpack_require__);
/******/ 	
/******/ 		// Return the exports of the module
/******/ 		return module.exports;
/******/ 	}
/******/ 	
/************************************************************************/
/******/ 	/* webpack/runtime/compat get default export */
/******/ 	(() => {
/******/ 		// getDefaultExport function for compatibility with non-harmony modules
/******/ 		__webpack_require__.n = (module) => {
/******/ 			var getter = module && module.__esModule ?
/******/ 				() => (module['default']) :
/******/ 				() => (module);
/******/ 			__webpack_require__.d(getter, { a: getter });
/******/ 			return getter;
/******/ 		};
/******/ 	})();
/******/ 	
/******/ 	/* webpack/runtime/define property getters */
/******/ 	(() => {
/******/ 		// define getter functions for harmony exports
/******/ 		__webpack_require__.d = (exports, definition) => {
/******/ 			for(var key in definition) {
/******/ 				if(__webpack_require__.o(definition, key) && !__webpack_require__.o(exports, key)) {
/******/ 					Object.defineProperty(exports, key, { enumerable: true, get: definition[key] });
/******/ 				}
/******/ 			}
/******/ 		};
/******/ 	})();
/******/ 	
/******/ 	/* webpack/runtime/hasOwnProperty shorthand */
/******/ 	(() => {
/******/ 		__webpack_require__.o = (obj, prop) => (Object.prototype.hasOwnProperty.call(obj, prop))
/******/ 	})();
/******/ 	
/******/ 	/* webpack/runtime/make namespace object */
/******/ 	(() => {
/******/ 		// define __esModule on exports
/******/ 		__webpack_require__.r = (exports) => {
/******/ 			if(typeof Symbol !== 'undefined' && Symbol.toStringTag) {
/******/ 				Object.defineProperty(exports, Symbol.toStringTag, { value: 'Module' });
/******/ 			}
/******/ 			Object.defineProperty(exports, '__esModule', { value: true });
/******/ 		};
/******/ 	})();
/******/ 	
/******/ 	/* webpack/runtime/nonce */
/******/ 	(() => {
/******/ 		__webpack_require__.nc = undefined;
/******/ 	})();
/******/ 	
/************************************************************************/
/******/ 	
/******/ 	// startup
/******/ 	// Load entry module and return exports
/******/ 	// This entry module can't be inlined because the eval devtool is used.
/******/ 	var __webpack_exports__ = __webpack_require__("./src/main.js");
/******/ 	
/******/ })()
;