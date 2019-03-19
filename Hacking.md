Hacking Tips
============

`window.gpu_fluid_main` holds the instance of `Main`.

This has all the shader objects and sources as its property.

The objects can be manipulated at run time to take effect so that we don`t have to touch the huge transpiled js file directly.


### Example: Updating GLSL on the Fly

```js

gpu_fluid_main.fluid.updateDyeShader._fragSource = `
//glsl code
`;

gpu_fluid_main.fluid.updateDyeShader.create();

```

#### Loading GLSL From File

```js
var resp = await fetch("shaders/glsl/fluid/advect.frag");
gpu_fluid_main.fluid.advectShader._fragSource = await resp.text();
gpu_fluid_main.fluid.updateDyeShader.create();
```

