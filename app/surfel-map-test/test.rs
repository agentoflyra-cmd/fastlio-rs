use anyhow::{Context, Result};
use camino::Utf8PathBuf;
use csv::{ReaderBuilder, Trim};
use fastlio_map::{surfel::SurfelMap, types::GeometryClass};
use fastlio_types::{Config, PointXYZI, Vec3};
use nalgebra::{Quaternion, UnitQuaternion};
use pcd_rs::{DataKind, PcdDeserialize, PcdSerialize, Reader, WriterInit};
use serde::Deserialize;
use std::{
    cmp::Ordering,
    env,
    fs::File,
    io::{BufReader, Read},
    time::Instant,
};
#[derive(Deserialize)]
struct SlamPose {
    #[serde(rename = "# counter")]
    counter: u64,
    sec: i64,
    nsec: u32,
    x: f32,
    y: f32,
    z: f32,
    qx: f32,
    qy: f32,
    qz: f32,
    qw: f32,
}

#[allow(dead_code)]
#[derive(PcdDeserialize)]
struct OxfordPcdPoint {
    x: f32,
    y: f32,
    z: f32,
    rgb: u32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    curvature: f32,
}

struct SurfelTestConfig {
    dataset_path: Utf8PathBuf,
    surfel_config_path: Utf8PathBuf,
    read_count: usize,
    output_path: Utf8PathBuf,
}

#[derive(PcdSerialize)]
struct SurfelPcdPoint {
    x: f32,
    y: f32,
    z: f32,
    intensity: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    class_id: f32,
}

#[derive(Default)]
struct LineClassificationDiagnostics {
    total: usize,
    failed_planarity_ratio: usize,
    failed_plane_spread: usize,
    failed_both: usize,
    planarity_ratio_min: Option<f64>,
    planarity_ratio_max: Option<f64>,
    lambda1_min: Option<f64>,
    lambda1_max: Option<f64>,
    lambda1_bins: [usize; 6],
    count_bins: [usize; 5],
    linearity_min: Option<f64>,
    linearity_max: Option<f64>,
    lambda0_over_lambda2_min: Option<f64>,
    lambda0_over_lambda2_max: Option<f64>,
    lambda2_min: Option<f64>,
    lambda2_max: Option<f64>,
    linearity_bins: [usize; 5],
}

impl LineClassificationDiagnostics {
    fn update_range(range: &mut (Option<f64>, Option<f64>), value: f64) {
        range.0 = Some(range.0.map_or(value, |current| current.min(value)));
        range.1 = Some(range.1.map_or(value, |current| current.max(value)));
    }

    fn collect(map: &SurfelMap) -> Self {
        let config = map.surfel_config();
        let mut diagnostics = Self::default();

        for surfel in map.surfels() {
            if !matches!(surfel.geometry_class(config), GeometryClass::Line) {
                continue;
            }

            let lambda_1 = surfel.eigenvalues[1];
            let planarity_ratio = if lambda_1 > 0.0 {
                surfel.eigenvalues[0] / lambda_1
            } else {
                f64::INFINITY
            };
            let failed_planarity_ratio = planarity_ratio
                .partial_cmp(&(config.max_planarity_ratio as f64))
                != Some(Ordering::Less);
            let failed_plane_spread = lambda_1
                .partial_cmp(&(config.min_plane_spread_eigenvalue as f64))
                != Some(Ordering::Greater);

            diagnostics.total += 1;
            diagnostics.failed_planarity_ratio += usize::from(failed_planarity_ratio);
            diagnostics.failed_plane_spread += usize::from(failed_plane_spread);
            diagnostics.failed_both += usize::from(failed_planarity_ratio && failed_plane_spread);

            let mut planarity_range = (
                diagnostics.planarity_ratio_min,
                diagnostics.planarity_ratio_max,
            );
            Self::update_range(&mut planarity_range, planarity_ratio);
            (
                diagnostics.planarity_ratio_min,
                diagnostics.planarity_ratio_max,
            ) = planarity_range;

            let mut lambda1_range = (diagnostics.lambda1_min, diagnostics.lambda1_max);
            Self::update_range(&mut lambda1_range, lambda_1);
            (diagnostics.lambda1_min, diagnostics.lambda1_max) = lambda1_range;

            let lambda1_bin = if lambda_1 <= 0.0005 {
                0
            } else if lambda_1 <= 0.001 {
                1
            } else if lambda_1 <= 0.0025 {
                2
            } else if lambda_1 <= 0.005 {
                3
            } else if lambda_1 <= 0.01 {
                4
            } else {
                5
            };
            diagnostics.lambda1_bins[lambda1_bin] += 1;

            let count_bin = match surfel.count {
                0..=8 => 0,
                9..=16 => 1,
                17..=32 => 2,
                33..=64 => 3,
                _ => 4,
            };
            diagnostics.count_bins[count_bin] += 1;

            let lambda_0 = surfel.eigenvalues[0];
            let lambda_2 = surfel.eigenvalues[2];
            let linearity = if lambda_2 > 0.0 {
                (lambda_2 - lambda_1) / lambda_2
            } else {
                f64::INFINITY
            };
            let lambda0_over_lambda2 = if lambda_2 > 0.0 {
                lambda_0 / lambda_2
            } else {
                f64::INFINITY
            };

            let mut linearity_range = (diagnostics.linearity_min, diagnostics.linearity_max);
            Self::update_range(&mut linearity_range, linearity);
            (diagnostics.linearity_min, diagnostics.linearity_max) = linearity_range;

            let mut scattering_range = (
                diagnostics.lambda0_over_lambda2_min,
                diagnostics.lambda0_over_lambda2_max,
            );
            Self::update_range(&mut scattering_range, lambda0_over_lambda2);
            (
                diagnostics.lambda0_over_lambda2_min,
                diagnostics.lambda0_over_lambda2_max,
            ) = scattering_range;

            let mut lambda2_range = (diagnostics.lambda2_min, diagnostics.lambda2_max);
            Self::update_range(&mut lambda2_range, lambda_2);
            (diagnostics.lambda2_min, diagnostics.lambda2_max) = lambda2_range;

            let linearity_bin = if linearity <= 0.5 {
                0
            } else if linearity <= 0.7 {
                1
            } else if linearity <= 0.8 {
                2
            } else if linearity <= 0.9 {
                3
            } else {
                4
            };
            diagnostics.linearity_bins[linearity_bin] += 1;
        }

        diagnostics
    }
}

impl SurfelTestConfig {
    fn from_args() -> Result<Self> {
        let mut args = env::args();
        let program = args.next().unwrap_or_else(|| "fastlio-surfel".to_string());
        let usage = || {
            format!(
                "usage: {program} <dataset_path> <surfel_config_path> [read_count] [output_path]"
            )
        };

        let dataset_path = Utf8PathBuf::from(args.next().context(usage())?);
        let surfel_config_path = Utf8PathBuf::from(args.next().context(usage())?);
        let read_count = args
            .next()
            .map(|value| {
                value
                    .parse::<usize>()
                    .context("read_count must be a non-negative integer")
            })
            .transpose()?
            .unwrap_or(100);
        let output_path =
            Utf8PathBuf::from(args.next().unwrap_or_else(|| "surfel-map.pcd".to_string()));
        if args.next().is_some() {
            anyhow::bail!(usage());
        }

        Ok(Self {
            dataset_path,
            surfel_config_path,
            read_count,
            output_path,
        })
    }
}

fn read_one_pcd(
    counter: u64,
    sec: i64,
    nsec: u32,
    config: &SurfelTestConfig,
) -> Result<impl Iterator<Item = PointXYZI>> {
    let filename = format!("cloud_{sec}_{nsec:09}.pcd");
    let path = config.dataset_path.join("undist-clouds").join(filename);
    let reader: Reader<OxfordPcdPoint, _> = Reader::open(&path)
        .with_context(|| format!("failed to open PCD `{path}` for pose counter {counter}"))?;
    let points: std::result::Result<Vec<OxfordPcdPoint>, _> = reader.collect();
    let points = points
        .with_context(|| format!("failed to decode PCD `{path}`"))?
        .into_iter()
        .map(|p| PointXYZI {
            x: p.x,
            y: p.y,
            z: p.z,
            intensity: 0.0,
        });
    Ok(points)
}

fn process_on_callback(config: &SurfelTestConfig) -> Result<SurfelMap> {
    let fastlio_config = read_to_surfel_config(config)?;
    let mut surfel_map = SurfelMap::new(
        fastlio_config.surfel_map_config.unwrap_or_default(),
        fastlio_config.surfel_config.unwrap_or_default(),
    );
    let csv_path = config.dataset_path.join("slam-poses.csv");
    let csv =
        File::open(&csv_path).with_context(|| format!("failed to open pose CSV `{csv_path}`"))?;
    let buf = BufReader::new(csv);
    let mut rdr = ReaderBuilder::new()
        .has_headers(true)
        .trim(Trim::All)
        .from_reader(buf);

    rdr.deserialize()
        .take(config.read_count)
        .enumerate()
        .try_for_each(|(index, record)| -> Result<()> {
            let slampose: SlamPose =
                record.with_context(|| format!("failed to decode pose CSV row {}", index + 2))?;
            let read_start = Instant::now();
            let points = read_one_pcd(slampose.counter, slampose.sec, slampose.nsec, config)?;
            let read_elapsed = read_start.elapsed();
            let t = Vec3::new(slampose.x, slampose.y, slampose.z);
            let quat = UnitQuaternion::from_quaternion(Quaternion::new(
                slampose.qw,
                slampose.qx,
                slampose.qy,
                slampose.qz,
            ));
            let r = quat.to_rotation_matrix().into_inner();
            let points: Vec<_> = points
                .map(|p| {
                    let p_new = r * p.to_vec3() + t;
                    PointXYZI {
                        x: p_new[0],
                        y: p_new[1],
                        z: p_new[2],
                        intensity: p.intensity,
                    }
                })
                .collect();
            let point_count = points.len();
            let transform_elapsed = read_start.elapsed() - read_elapsed;
            let insert_start = Instant::now();
            surfel_map.insert(points.into_iter())?;
            let insert_elapsed = insert_start.elapsed();
            eprintln!(
                "callback counter={} points={} read={:?} transform={:?} insert={:?} total={:?}",
                slampose.counter,
                point_count,
                read_elapsed,
                transform_elapsed,
                insert_elapsed,
                read_start.elapsed()
            );
            Ok(())
        })?;
    Ok(surfel_map)
}

fn write_pcd(
    path: &Utf8PathBuf,
    width: usize,
    points: impl IntoIterator<Item = SurfelPcdPoint>,
) -> Result<()> {
    let mut writer = WriterInit {
        width: width as u64,
        height: 1,
        viewpoint: Default::default(),
        data_kind: DataKind::Binary,
        schema: None,
        version: None,
    }
    .create::<SurfelPcdPoint, _>(path)?;

    for point in points {
        writer.push(&point)?;
    }
    writer.finish()?;
    Ok(())
}

fn write_surfel_map(map: &SurfelMap, path: &Utf8PathBuf) -> Result<()> {
    let width = map.surfels().count();
    let surfel_config = map.surfel_config();
    write_pcd(
        path,
        width,
        map.surfels().map(|surfel| SurfelPcdPoint {
            x: surfel.mean_w.x as f32,
            y: surfel.mean_w.y as f32,
            z: surfel.mean_w.z as f32,
            intensity: surfel.count as f32,
            normal_x: surfel.eigenvectors[(0, 0)] as f32,
            normal_y: surfel.eigenvectors[(1, 0)] as f32,
            normal_z: surfel.eigenvectors[(2, 0)] as f32,
            class_id: match surfel.geometry_class(surfel_config) {
                GeometryClass::Plane => 0.0,
                GeometryClass::Line => 1.0,
                GeometryClass::Scatter => 2.0,
                GeometryClass::Degenerate => 3.0,
                GeometryClass::Growing => 4.0,
            },
        }),
    )
}

fn print_line_classification_diagnostics(map: &SurfelMap) {
    let diagnostics = LineClassificationDiagnostics::collect(map);
    println!(
        "line diagnostics: total={} failed_planarity_ratio={} failed_plane_spread={} failed_both={}",
        diagnostics.total,
        diagnostics.failed_planarity_ratio,
        diagnostics.failed_plane_spread,
        diagnostics.failed_both,
    );
    println!(
        "line diagnostics: planarity_ratio={:?}..{:?} lambda1={:?}..{:?}",
        diagnostics.planarity_ratio_min,
        diagnostics.planarity_ratio_max,
        diagnostics.lambda1_min,
        diagnostics.lambda1_max,
    );
    println!(
        "line diagnostics: lambda1_bins=[<=0.0005:{}, <=0.001:{}, <=0.0025:{}, <=0.005:{}, <=0.01:{}, >0.01:{}]",
        diagnostics.lambda1_bins[0],
        diagnostics.lambda1_bins[1],
        diagnostics.lambda1_bins[2],
        diagnostics.lambda1_bins[3],
        diagnostics.lambda1_bins[4],
        diagnostics.lambda1_bins[5],
    );
    println!(
        "line diagnostics: count_bins=[<=8:{}, 9..16:{}, 17..32:{}, 33..64:{}, >64:{}]",
        diagnostics.count_bins[0],
        diagnostics.count_bins[1],
        diagnostics.count_bins[2],
        diagnostics.count_bins[3],
        diagnostics.count_bins[4],
    );
    println!(
        "line diagnostics: linearity={:?}..{:?} lambda0/lambda2={:?}..{:?} lambda2={:?}..{:?}",
        diagnostics.linearity_min,
        diagnostics.linearity_max,
        diagnostics.lambda0_over_lambda2_min,
        diagnostics.lambda0_over_lambda2_max,
        diagnostics.lambda2_min,
        diagnostics.lambda2_max,
    );
    println!(
        "line diagnostics: linearity_bins=[<=0.5:{}, <=0.7:{}, <=0.8:{}, <=0.9:{}, >0.9:{}]",
        diagnostics.linearity_bins[0],
        diagnostics.linearity_bins[1],
        diagnostics.linearity_bins[2],
        diagnostics.linearity_bins[3],
        diagnostics.linearity_bins[4],
    );
}

fn read_to_surfel_config(config: &SurfelTestConfig) -> Result<Config> {
    let mut buf = String::new();
    let _ = File::open(&config.surfel_config_path)
        .expect("config should be exist.")
        .read_to_string(&mut buf);
    let config: Config = serde_yaml::from_str(&buf).expect("serde failed.");
    Ok(config)
}

fn main() -> Result<()> {
    let config = SurfelTestConfig::from_args()?;
    let surfel_map = process_on_callback(&config)?;
    print_line_classification_diagnostics(&surfel_map);
    let write_start = Instant::now();
    write_surfel_map(&surfel_map, &config.output_path)?;
    let write_elapsed = write_start.elapsed();
    println!(
        "wrote {} surfels to {}",
        surfel_map.surfels().count(),
        config.output_path
    );
    eprintln!("write_map elapsed={write_elapsed:?}");
    Ok(())
}
