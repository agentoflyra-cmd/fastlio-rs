struct Uniforms {
    view: array<vec4<f32>, 4>,
    viewport: vec2<f32>,
    scale: f32,
    radius: f32,
};

struct Point {
    position: vec3<f32>,
    intensity: f32,
    color: vec3<f32>,
    radius: f32,
};

struct VertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) splat_offset: vec2<f32>,
};

@group(0) @binding(0)
var<uniform> uniforms: Uniforms;

@vertex
fn vs_main(
    @location(0) position: vec3<f32>,
    @location(1) intensity: f32,
    @location(2) color: vec3<f32>,
    @location(3) radius: f32,
    @builtin(vertex_index) vertex_index: u32,
) -> VertexOutput {
    let offsets = array<vec2<f32>, 6>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>(1.0, -1.0),
        vec2<f32>(1.0, 1.0),
        vec2<f32>(-1.0, -1.0),
        vec2<f32>(1.0, 1.0),
        vec2<f32>(-1.0, 1.0),
    );

    let world_position = vec4<f32>(position, 1.0);
    let view_position = vec4<f32>(
        dot(uniforms.view[0], world_position),
        dot(uniforms.view[1], world_position),
        dot(uniforms.view[2], world_position),
        dot(uniforms.view[3], world_position),
    );
    let brightness = clamp(intensity, 0.15, 1.0);
    var output: VertexOutput;
    output.color = vec4<f32>(color * brightness, 1.0);
    output.splat_offset = offsets[vertex_index];

    let center = uniforms.viewport * 0.5;
    let screen = vec2<f32>(
        center.x + view_position.x * uniforms.scale,
        center.y - view_position.y * uniforms.scale,
    ) + offsets[vertex_index] * max(radius, uniforms.radius);

    let ndc = vec2<f32>(
        screen.x / uniforms.viewport.x * 2.0 - 1.0,
        1.0 - screen.y / uniforms.viewport.y * 2.0,
    );

    output.position = vec4<f32>(ndc, 0.0, 1.0);
    return output;
}

@fragment
fn fs_main(input: VertexOutput) -> @location(0) vec4<f32> {
    let dist2 = dot(input.splat_offset, input.splat_offset);

    if (dist2 > 1.0) {
        discard;
    }

    let alpha = exp(-dist2 * 2.8) * input.color.a;
    return vec4<f32>(input.color.rgb, alpha);
}
