var gl =  snow_modules_opengl_web_GL.gl;
var gpu_fluid_main = window.gpu_fluid_main;

function deleteShaderProgram (shader) {
    gl.deleteProgram(shader._prog);
}

var tex = gl.createTexture();

gl.bindTexture(gl.TEXTURE_2D, tex);

//gltoolbox_shaders_Resample.instance._active

function reveal (xs) {
    var xs = Array.prototype.slice.call(xs);
    return xs.flat().flatMap((x)=>{
        if (typeof x == "function") return x();
        else return x;
    });
}

function findObjectsOfClass (root, className, depth) {
    return findObjectsOfClassInner(root, className, depth, []).flat();
}

function findObjectsOfClassInner(root, className, depth, looked) {
    if (looked.indexOf(root) > -1) return []; //cycle detection
    if (depth <= 0) return [];
    looked.push(root);
    return Object.entries(root).map(([key,val]) => {
        if (val && typeof val === "object") {
            console.log(val.constructor.name);
            if (val.constructor.name == className)
                return [{val: val, parent: root}];
            else
                return ()=> findObjectsOfClassInner(val, className, depth-1, looked);
        } else if (typeof val == className)
            return [{val: val, parent: root}];
        else return [];
    });
}

var xs = findObjectsOfClass(gpu_fluid_main, "Float32Array", 10);
xs
xs = reveal(xs)
xs
xs = reveal(xs)
