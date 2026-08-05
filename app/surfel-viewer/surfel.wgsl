struct Uniforms { view: array<vec4<f32>, 4>, viewport: vec2<f32>, scale: f32, padding: f32 };
struct Output { @builtin(position) position: vec4<f32>, @location(0) color: vec4<f32>, @location(1) offset: vec2<f32> };
@group(0) @binding(0) var<uniform> uniforms: Uniforms;
@vertex fn vs_main(@location(0) position: vec3<f32>, @location(1) color: vec3<f32>, @location(2) radius: f32, @builtin(vertex_index) index: u32) -> Output {
  let offsets = array<vec2<f32>, 6>(vec2(-1.0,-1.0), vec2(1.0,-1.0), vec2(1.0,1.0), vec2(-1.0,-1.0), vec2(1.0,1.0), vec2(-1.0,1.0));
  let p = vec4<f32>(position, 1.0); let v = vec4(dot(uniforms.view[0],p), dot(uniforms.view[1],p), dot(uniforms.view[2],p), dot(uniforms.view[3],p));
  let center = uniforms.viewport * 0.5; let screen = center + vec2(v.x, -v.y) * uniforms.scale + offsets[index] * radius;
  var out: Output; out.position = vec4(screen.x / uniforms.viewport.x * 2.0 - 1.0, 1.0 - screen.y / uniforms.viewport.y * 2.0, 0.0, 1.0); out.color = vec4(color, 0.9); out.offset = offsets[index]; return out;
}
@fragment fn fs_main(input: Output) -> @location(0) vec4<f32> { let d = dot(input.offset, input.offset); if (d > 1.0) { discard; } return vec4(input.color.rgb, input.color.a * exp(-d * 2.8)); }
