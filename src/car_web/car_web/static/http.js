(function installHttpClient(global) {
  "use strict";

  async function api(path, options = {}) {
    const response = await fetch(path, {
      headers: { "Content-Type": "application/json" },
      ...options,
    });
    const responseText = await response.text();
    let payload = {};
    if (responseText) {
      try {
        payload = JSON.parse(responseText);
      } catch (_) {
        if (!response.ok) {
          throw new Error(`服务请求失败，HTTP ${response.status}`);
        }
        throw new Error("服务返回数据格式错误");
      }
    }
    if (!response.ok || payload.ok === false) {
      throw new Error(payload.detail || payload.message || `HTTP ${response.status}`);
    }
    return payload;
  }

  function post(path, body = {}) {
    return api(path, {
      method: "POST",
      body: JSON.stringify(body),
    });
  }

  global.CarHttp = Object.freeze({ api, post });
}(window));
