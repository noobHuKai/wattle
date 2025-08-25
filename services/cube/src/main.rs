use rerun::{demo_util::grid, external::glam};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 保存到文件而不是启动服务器
    let rec = rerun::RecordingStreamBuilder::new("rerun_example_minimal")
        .save("cube_visualization.rrd")?;

    let points = grid(glam::Vec3::splat(-10.0), glam::Vec3::splat(10.0), 10);
    let colors = grid(glam::Vec3::ZERO, glam::Vec3::splat(255.0), 10)
        .map(|v| rerun::Color::from_rgb(v.x as u8, v.y as u8, v.z as u8));

    rec.log(
        "my_points",
        &rerun::Points3D::new(points)
            .with_colors(colors)
            .with_radii([0.5]),
    )?;

    println!("✅ 成功生成 3D 点云数据并保存到 cube_visualization.rrd");
    println!("🎯 你可以使用 'rerun cube_visualization.rrd' 来查看可视化结果");

    Ok(())
}