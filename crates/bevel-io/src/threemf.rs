// SPDX-License-Identifier: GPL-3.0-or-later

//! 3MF reading and writing.
//!
//! Follows the 3MF Core Specification v1.3 (3MF Consortium), which packages
//! a model with the Open Packaging Conventions: a ZIP archive holding
//! `[Content_Types].xml`, `_rels/.rels` and the model part
//! `3D/3dmodel.model`.
//!
//! # Writing
//!
//! The model uses `unit="millimeter"`. Each [`ExportMesh`] becomes one
//! `<object type="model">` carrying its name, a `<mesh>` with vertices
//! welded per object (positions equal bit for bit become one vertex, so the
//! file is a proper indexed mesh) and its triangles, and one `<item>` in
//! `<build>`. Colours go into a single `<basematerials>` group as
//! `displaycolor` (`#RRGGBB`, or `#RRGGBBAA` when not opaque); each coloured
//! object points at its entry with `pid` and `pindex`. Coordinates are
//! written with Rust's shortest round-trip `f64` formatting, so reading them
//! back gives the same values exactly.
//!
//! The output is deterministic: entries are stored in a fixed order with a
//! fixed modification time (1980-01-01, the ZIP epoch), no map iteration
//! order reaches the file, and identical input gives identical bytes.
//!
//! # Reading
//!
//! The reader handles what this writer produces and simple files from other
//! tools: the `unit` attribute (converted to millimetres), `basematerials`
//! colours referenced by an object's `pid`/`pindex`, `<build>` items in
//! order including their `transform`, and objects without a build section
//! (all model objects are returned then). Component assemblies, per-triangle
//! materials and extension data are not supported; assemblies are reported
//! as an error rather than silently dropped.

use std::collections::BTreeMap;
use std::io::{Cursor, Read, Seek, Write};

use quick_xml::Writer;
use quick_xml::events::{BytesDecl, BytesEnd, BytesStart, Event};
use quick_xml::{Reader, XmlVersion};
use zip::write::SimpleFileOptions;
use zip::{CompressionMethod, DateTime, ZipArchive, ZipWriter};

use crate::mesh::Welded;
use crate::{ExportMesh, IoError};

const CORE_NS: &str = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02";
const MODEL_PATH: &str = "3D/3dmodel.model";
const MODEL_REL_TYPE: &str = "http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel";
/// Largest package part the reader will inflate, as a guard against
/// decompression bombs.
const MAX_PART_BYTES: u64 = 1 << 30;

const CONTENT_TYPES: &str = concat!(
    r#"<?xml version="1.0" encoding="UTF-8"?>"#,
    "\n",
    r#"<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">"#,
    r#"<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>"#,
    r#"<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>"#,
    "</Types>\n",
);

const RELATIONSHIPS: &str = concat!(
    r#"<?xml version="1.0" encoding="UTF-8"?>"#,
    "\n",
    r#"<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">"#,
    r#"<Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>"#,
    "</Relationships>\n",
);

/// Writes the meshes as a 3MF package.
///
/// See the module documentation for the layout. Coordinates are millimetres.
///
/// # Errors
///
/// [`IoError::InvalidMesh`] if there are no meshes, a mesh fails
/// [`ExportMesh::validate`], or a triangle collapses (two of its vertices
/// sit at the same position); [`IoError::Zip`] or [`IoError::Io`] if
/// writing fails.
pub fn write(meshes: &[ExportMesh], out: &mut (impl Write + Seek)) -> Result<(), IoError> {
    let model = model_xml(meshes)?;
    let options = SimpleFileOptions::default()
        .compression_method(CompressionMethod::Deflated)
        .last_modified_time(DateTime::default())
        .unix_permissions(0o644);
    let mut zip = ZipWriter::new(out);
    zip.start_file("[Content_Types].xml", options)?;
    zip.write_all(CONTENT_TYPES.as_bytes())?;
    zip.start_file("_rels/.rels", options)?;
    zip.write_all(RELATIONSHIPS.as_bytes())?;
    zip.start_file(MODEL_PATH, options)?;
    zip.write_all(&model)?;
    zip.finish()?;
    Ok(())
}

/// Reads the meshes of a 3MF package.
///
/// Positions come back in millimetres, in build order. Objects the file
/// lists in `<build>` more than once are returned once per item. See the
/// module documentation for what is supported.
///
/// # Errors
///
/// [`IoError::Zip`] if the archive is damaged, [`IoError::Xml`] if a part is
/// not well-formed XML, and [`IoError::InvalidFile`] if the content breaks
/// the format (missing model part, bad numbers, indices out of range,
/// unsupported components).
pub fn read(bytes: &[u8]) -> Result<Vec<ExportMesh>, IoError> {
    let mut archive = ZipArchive::new(Cursor::new(bytes))?;
    let model_path = match read_part(&mut archive, "_rels/.rels") {
        Ok(rels) => model_path_from_rels(&rels)?.unwrap_or_else(|| MODEL_PATH.to_owned()),
        Err(_) => MODEL_PATH.to_owned(),
    };
    let model = read_part(&mut archive, &model_path)?;
    parse_model(&model)
}

// ---------------------------------------------------------------- writing

/// The text of `3D/3dmodel.model` for the given meshes.
pub(crate) fn model_xml(meshes: &[ExportMesh]) -> Result<Vec<u8>, IoError> {
    if meshes.is_empty() {
        return Err(IoError::mesh("", "there is nothing to export"));
    }
    // Weld first so every error is reported before any bytes are produced.
    let mut welded = Vec::with_capacity(meshes.len());
    for m in meshes {
        m.validate()?;
        let (positions, triangles) = m
            .welded()
            .ok_or_else(|| IoError::mesh(&m.name, "the mesh is too large for 3MF"))?;
        if let Some(i) = triangles
            .iter()
            .position(|t| t[0] == t[1] || t[1] == t[2] || t[0] == t[2])
        {
            return Err(IoError::mesh(
                &m.name,
                format!("triangle {i} has two corners at the same position"),
            ));
        }
        welded.push((positions, triangles));
    }

    // Distinct colours in order of first use.
    let mut colors: Vec<[u8; 4]> = Vec::new();
    for c in meshes.iter().filter_map(|m| m.color) {
        if !colors.contains(&c) {
            colors.push(c);
        }
    }

    let mut w = Writer::new_with_indent(Vec::new(), b' ', 2);
    w.write_event(Event::Decl(BytesDecl::new("1.0", Some("UTF-8"), None)))?;
    let mut model = BytesStart::new("model");
    model.push_attribute(("unit", "millimeter"));
    model.push_attribute(("xml:lang", "en-US"));
    model.push_attribute(("xmlns", CORE_NS));
    w.write_event(Event::Start(model))?;
    open(&mut w, "resources", &[])?;

    let mut next_id = 1u32;
    let materials_id = next_id;
    if !colors.is_empty() {
        next_id += 1;
        open(
            &mut w,
            "basematerials",
            &[("id", &materials_id.to_string())],
        )?;
        for (i, c) in colors.iter().enumerate() {
            let name = format!("Color {}", i + 1);
            empty(
                &mut w,
                "base",
                &[("name", &name), ("displaycolor", &hex_color(*c))],
            )?;
        }
        close(&mut w, "basematerials")?;
    }

    let first_object_id = next_id;
    for (k, (m, welded_mesh)) in meshes.iter().zip(&welded).enumerate() {
        let id = object_id(first_object_id, k)?;
        let color_ref = m
            .color
            .and_then(|c| colors.iter().position(|x| *x == c))
            .map(|index| (materials_id, index));
        write_object(&mut w, id, m, welded_mesh, color_ref)?;
    }
    close(&mut w, "resources")?;

    open(&mut w, "build", &[])?;
    for k in 0..meshes.len() {
        let id = object_id(first_object_id, k)?.to_string();
        empty(&mut w, "item", &[("objectid", &id)])?;
    }
    close(&mut w, "build")?;
    close(&mut w, "model")?;
    let mut bytes = w.into_inner();
    bytes.push(b'\n');
    Ok(bytes)
}

fn write_object(
    w: &mut Writer<Vec<u8>>,
    id: u32,
    m: &ExportMesh,
    (positions, triangles): &Welded,
    color_ref: Option<(u32, usize)>,
) -> Result<(), IoError> {
    let mut attrs: Vec<(&str, String)> = vec![
        ("id", id.to_string()),
        ("name", clean_name(&m.name)),
        ("type", "model".to_owned()),
    ];
    if let Some((group, index)) = color_ref {
        attrs.push(("pid", group.to_string()));
        attrs.push(("pindex", index.to_string()));
    }
    let borrowed: Vec<(&str, &str)> = attrs.iter().map(|(k, v)| (*k, v.as_str())).collect();
    open(w, "object", &borrowed)?;
    open(w, "mesh", &[])?;
    open(w, "vertices", &[])?;
    for p in positions {
        let (x, y, z) = (p[0].to_string(), p[1].to_string(), p[2].to_string());
        empty(w, "vertex", &[("x", &x), ("y", &y), ("z", &z)])?;
    }
    close(w, "vertices")?;
    open(w, "triangles", &[])?;
    for t in triangles {
        let (a, b, c) = (t[0].to_string(), t[1].to_string(), t[2].to_string());
        empty(w, "triangle", &[("v1", &a), ("v2", &b), ("v3", &c)])?;
    }
    close(w, "triangles")?;
    close(w, "mesh")?;
    close(w, "object")
}

fn object_id(first: u32, k: usize) -> Result<u32, IoError> {
    u32::try_from(k)
        .ok()
        .and_then(|k| first.checked_add(k))
        .ok_or_else(|| IoError::mesh("", "there are too many meshes for 3MF"))
}

fn open(w: &mut Writer<Vec<u8>>, name: &str, attrs: &[(&str, &str)]) -> Result<(), IoError> {
    let mut e = BytesStart::new(name);
    for a in attrs {
        e.push_attribute(*a);
    }
    w.write_event(Event::Start(e))?;
    Ok(())
}

fn empty(w: &mut Writer<Vec<u8>>, name: &str, attrs: &[(&str, &str)]) -> Result<(), IoError> {
    let mut e = BytesStart::new(name);
    for a in attrs {
        e.push_attribute(*a);
    }
    w.write_event(Event::Empty(e))?;
    Ok(())
}

fn close(w: &mut Writer<Vec<u8>>, name: &str) -> Result<(), IoError> {
    w.write_event(Event::End(BytesEnd::new(name)))?;
    Ok(())
}

/// Replaces control characters, which XML cannot carry, with spaces.
fn clean_name(name: &str) -> String {
    name.chars()
        .map(|c| if c.is_control() { ' ' } else { c })
        .collect()
}

fn hex_color(c: [u8; 4]) -> String {
    if c[3] == 255 {
        format!("#{:02X}{:02X}{:02X}", c[0], c[1], c[2])
    } else {
        format!("#{:02X}{:02X}{:02X}{:02X}", c[0], c[1], c[2], c[3])
    }
}

// ---------------------------------------------------------------- reading

fn read_part(archive: &mut ZipArchive<Cursor<&[u8]>>, name: &str) -> Result<Vec<u8>, IoError> {
    let file = archive.by_name(name)?;
    let mut data = Vec::new();
    file.take(MAX_PART_BYTES + 1).read_to_end(&mut data)?;
    if data.len() as u64 > MAX_PART_BYTES {
        return Err(IoError::file(
            "3MF",
            format!("the part {name} is too large"),
        ));
    }
    Ok(data)
}

fn attr(e: &BytesStart<'_>, key: &str) -> Result<Option<String>, IoError> {
    for a in e.attributes() {
        let a = a.map_err(quick_xml::Error::from)?;
        if a.key.local_name().as_ref() == key {
            return Ok(Some(
                a.normalized_value(XmlVersion::Implicit1_0)?.into_owned(),
            ));
        }
    }
    Ok(None)
}

fn model_path_from_rels(rels: &[u8]) -> Result<Option<String>, IoError> {
    let mut reader = Reader::from_reader(rels);
    loop {
        match reader.read_event()? {
            Event::Start(e) | Event::Empty(e) if e.local_name().as_ref() == "Relationship" => {
                let kind = attr(&e, "Type")?.unwrap_or_default();
                let is_model = kind == MODEL_REL_TYPE || kind.ends_with("/3dmodel");
                if let (true, Some(target)) = (is_model, attr(&e, "Target")?) {
                    return Ok(Some(target.trim_start_matches('/').to_owned()));
                }
            }
            Event::Eof => return Ok(None),
            _ => {}
        }
    }
}

fn parse_number<T: std::str::FromStr>(text: &str, what: &str) -> Result<T, IoError> {
    text.trim()
        .parse()
        .map_err(|_| IoError::file("3MF", format!("\"{text}\" is not a valid {what}")))
}

fn required<T: std::str::FromStr>(e: &BytesStart<'_>, key: &str, what: &str) -> Result<T, IoError> {
    let text = attr(e, key)?
        .ok_or_else(|| IoError::file("3MF", format!("a <{}> has no {key}", element_name(e))))?;
    parse_number(&text, what)
}

fn element_name(e: &BytesStart<'_>) -> String {
    e.local_name().as_ref().to_owned()
}

fn parse_color(text: &str) -> Option<[u8; 4]> {
    let hex = text.trim().strip_prefix('#')?;
    if !hex.is_ascii() || (hex.len() != 6 && hex.len() != 8) {
        return None;
    }
    let byte = |i: usize| u8::from_str_radix(&hex[i..i + 2], 16).ok();
    let alpha = if hex.len() == 8 { byte(6)? } else { 255 };
    Some([byte(0)?, byte(2)?, byte(4)?, alpha])
}

fn unit_scale(unit: &str) -> Result<f64, IoError> {
    Ok(match unit {
        "micron" => 1e-3,
        "millimeter" => 1.0,
        "centimeter" => 10.0,
        "inch" => 25.4,
        "foot" => 304.8,
        "meter" => 1000.0,
        other => {
            return Err(IoError::file("3MF", format!("unknown unit \"{other}\"")));
        }
    })
}

#[derive(Default)]
struct ObjectData {
    id: u32,
    name: String,
    is_model: bool,
    pid: Option<u32>,
    pindex: usize,
    has_components: bool,
    positions: Vec<[f64; 3]>,
    triangles: Vec<[u32; 3]>,
}

struct Item {
    object: u32,
    transform: Option<[f64; 12]>,
}

#[derive(Default)]
struct ModelData {
    scale: f64,
    materials: BTreeMap<u32, Vec<Option<[u8; 4]>>>,
    objects: Vec<ObjectData>,
    items: Vec<Item>,
}

fn parse_model(xml: &[u8]) -> Result<Vec<ExportMesh>, IoError> {
    let mut reader = Reader::from_reader(xml);
    reader.config_mut().trim_text(true);
    let mut data = ModelData {
        scale: 1.0,
        ..ModelData::default()
    };
    let mut current_group: Option<u32> = None;
    let mut current_object: Option<ObjectData> = None;
    let mut seen_model = false;

    loop {
        let event = reader.read_event()?;
        match event {
            Event::Start(ref e) | Event::Empty(ref e) => {
                let name = e.local_name();
                match name.as_ref() {
                    "model" => {
                        seen_model = true;
                        if let Some(unit) = attr(e, "unit")? {
                            data.scale = unit_scale(&unit)?;
                        }
                    }
                    "basematerials" => {
                        let id = required(e, "id", "id")?;
                        data.materials.entry(id).or_default();
                        current_group = Some(id);
                    }
                    "base" => {
                        if let Some(group) = current_group {
                            let color = attr(e, "displaycolor")?.and_then(|t| parse_color(&t));
                            data.materials.entry(group).or_default().push(color);
                        }
                    }
                    "object" => current_object = Some(start_object(e)?),
                    "components" => {
                        if let Some(o) = current_object.as_mut() {
                            o.has_components = true;
                        }
                    }
                    "vertex" => {
                        if let Some(o) = current_object.as_mut() {
                            o.positions.push([
                                required(e, "x", "coordinate")?,
                                required(e, "y", "coordinate")?,
                                required(e, "z", "coordinate")?,
                            ]);
                        }
                    }
                    "triangle" => {
                        if let Some(o) = current_object.as_mut() {
                            o.triangles.push([
                                required(e, "v1", "vertex index")?,
                                required(e, "v2", "vertex index")?,
                                required(e, "v3", "vertex index")?,
                            ]);
                        }
                    }
                    "item" => data.items.push(start_item(e)?),
                    _ => {}
                }
            }
            Event::End(ref e) => match e.local_name().as_ref() {
                "basematerials" => current_group = None,
                "object" => {
                    if let Some(o) = current_object.take() {
                        data.objects.push(o);
                    }
                }
                _ => {}
            },
            Event::Eof => break,
            _ => {}
        }
    }
    if !seen_model {
        return Err(IoError::file(
            "3MF",
            "the model part has no <model> element",
        ));
    }
    assemble(&data)
}

fn start_object(e: &BytesStart<'_>) -> Result<ObjectData, IoError> {
    let kind = attr(e, "type")?.unwrap_or_else(|| "model".to_owned());
    let pid = attr(e, "pid")?
        .map(|t| parse_number(&t, "property id"))
        .transpose()?;
    let pindex = attr(e, "pindex")?
        .map(|t| parse_number(&t, "property index"))
        .transpose()?
        .unwrap_or(0);
    Ok(ObjectData {
        id: required(e, "id", "id")?,
        name: attr(e, "name")?.unwrap_or_default(),
        is_model: kind == "model",
        pid,
        pindex,
        ..ObjectData::default()
    })
}

fn start_item(e: &BytesStart<'_>) -> Result<Item, IoError> {
    let transform = match attr(e, "transform")? {
        None => None,
        Some(text) => {
            let values: Vec<f64> = text
                .split_whitespace()
                .map(|t| parse_number(t, "transform value"))
                .collect::<Result<_, _>>()?;
            let matrix: [f64; 12] = values
                .try_into()
                .map_err(|_| IoError::file("3MF", "a transform must have 12 numbers"))?;
            Some(matrix)
        }
    };
    Ok(Item {
        object: required(e, "objectid", "object id")?,
        transform,
    })
}

fn assemble(data: &ModelData) -> Result<Vec<ExportMesh>, IoError> {
    let mut out = Vec::new();
    if data.items.is_empty() {
        for o in data.objects.iter().filter(|o| o.is_model) {
            if let Some(m) = build_mesh(data, o, None)? {
                out.push(m);
            }
        }
    } else {
        for item in &data.items {
            let o = data
                .objects
                .iter()
                .find(|o| o.id == item.object)
                .ok_or_else(|| {
                    IoError::file(
                        "3MF",
                        format!("a build item refers to missing object {}", item.object),
                    )
                })?;
            if !o.is_model {
                continue;
            }
            if let Some(m) = build_mesh(data, o, item.transform.as_ref())? {
                out.push(m);
            }
        }
    }
    Ok(out)
}

fn build_mesh(
    data: &ModelData,
    o: &ObjectData,
    transform: Option<&[f64; 12]>,
) -> Result<Option<ExportMesh>, IoError> {
    if o.has_components {
        return Err(IoError::file(
            "3MF",
            format!(
                "object {} is an assembly of components, which is not supported",
                o.id
            ),
        ));
    }
    if o.positions.is_empty() && o.triangles.is_empty() {
        return Ok(None);
    }
    let color = o
        .pid
        .and_then(|pid| data.materials.get(&pid))
        .and_then(|group| group.get(o.pindex))
        .copied()
        .flatten();
    let mut mesh = ExportMesh {
        name: o.name.clone(),
        color,
        positions: o.positions.clone(),
        triangles: o.triangles.clone(),
    };
    if let Some(m) = transform {
        for p in &mut mesh.positions {
            let [x, y, z] = *p;
            *p = [
                x * m[0] + y * m[3] + z * m[6] + m[9],
                x * m[1] + y * m[4] + z * m[7] + m[10],
                x * m[2] + y * m[5] + z * m[8] + m[11],
            ];
        }
        let det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6])
            + m[2] * (m[3] * m[7] - m[4] * m[6]);
        if det < 0.0 {
            // A mirroring transform reverses the winding.
            for t in &mut mesh.triangles {
                t.swap(1, 2);
            }
        }
    }
    if (data.scale - 1.0).abs() > f64::EPSILON {
        for p in &mut mesh.positions {
            *p = p.map(|c| c * data.scale);
        }
    }
    mesh.validate_indices_and_coords()?;
    Ok(Some(mesh))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_util::unit_cube;

    /// Triangles as corner coordinates, independent of vertex numbering.
    pub(crate) fn resolved(m: &ExportMesh) -> Vec<[[f64; 3]; 3]> {
        m.triangles
            .iter()
            .map(|t| t.map(|v| m.positions[v as usize]))
            .collect()
    }

    fn write_to_vec(meshes: &[ExportMesh]) -> Vec<u8> {
        let mut cursor = Cursor::new(Vec::new());
        write(meshes, &mut cursor).unwrap();
        cursor.into_inner()
    }

    #[test]
    fn cube_round_trip() {
        let cube = unit_cube();
        let back = read(&write_to_vec(std::slice::from_ref(&cube))).unwrap();
        assert_eq!(back.len(), 1);
        let b = &back[0];
        assert_eq!(b.name, "Cube");
        assert_eq!(b.color, cube.color);
        assert_eq!(b.positions.len(), 8);
        assert_eq!(resolved(b), resolved(&cube));
        assert!(b.is_watertight());
    }

    #[test]
    fn package_has_the_required_parts() {
        let bytes = write_to_vec(&[unit_cube()]);
        let mut archive = ZipArchive::new(Cursor::new(bytes.as_slice())).unwrap();
        let mut names: Vec<String> = archive
            .file_names()
            .map(|n| n.unwrap().into_owned())
            .collect();
        names.sort();
        assert_eq!(
            names,
            ["3D/3dmodel.model", "[Content_Types].xml", "_rels/.rels"]
        );
        let model = read_part(&mut archive, MODEL_PATH).unwrap();
        let text = String::from_utf8(model).unwrap();
        assert!(text.contains(r#"unit="millimeter""#));
        assert!(text.contains(r#"<object id="2" name="Cube" type="model" pid="1" pindex="0">"#));
    }

    #[test]
    fn output_is_byte_identical_across_runs() {
        let meshes = [unit_cube(), unit_cube()];
        assert_eq!(write_to_vec(&meshes), write_to_vec(&meshes));
    }

    #[test]
    fn colours_are_shared_and_alpha_is_kept() {
        let mut a = unit_cube();
        a.name = "A".to_owned();
        a.color = Some([1, 2, 3, 255]);
        let mut b = a.clone();
        b.name = "B".to_owned();
        b.color = Some([255, 0, 16, 128]);
        let mut c = a.clone();
        c.name = "C & <D>".to_owned();
        c.color = None;
        let back = read(&write_to_vec(&[a.clone(), b.clone(), a.clone(), c])).unwrap();
        let colors: Vec<_> = back.iter().map(|m| m.color).collect();
        assert_eq!(colors, [a.color, b.color, a.color, None]);
        assert_eq!(back[3].name, "C & <D>");
    }

    #[test]
    fn several_objects_keep_their_order() {
        let mut a = unit_cube();
        a.name = "first".to_owned();
        let mut b = unit_cube();
        b.name = "second".to_owned();
        for p in &mut b.positions {
            p[0] += 10.0;
        }
        let back = read(&write_to_vec(&[a, b])).unwrap();
        assert_eq!(back[0].name, "first");
        assert_eq!(back[1].name, "second");
        assert!(back[1].positions.iter().all(|p| p[0] >= 10.0));
    }

    #[test]
    fn writer_rejects_bad_meshes() {
        let mut cursor = Cursor::new(Vec::new());
        assert!(write(&[], &mut cursor).is_err());
        let mut bad = unit_cube();
        bad.triangles[0][0] = 40;
        assert!(write(&[bad], &mut cursor).is_err());
        let mut collapsed = unit_cube();
        collapsed.positions[1] = collapsed.positions[0];
        assert!(write(&[collapsed], &mut cursor).is_err());
    }

    #[test]
    fn reader_rejects_garbage() {
        assert!(read(b"not a zip").is_err());
        assert!(read(&[]).is_err());
    }

    const FOREIGN: &str = r##"<?xml version="1.0"?>
<model unit="inch" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
 <resources>
  <basematerials id="5"><base name="x" displaycolor="#0000FF"/></basematerials>
  <object id="7" type="model" name="part" pid="5" pindex="0">
   <mesh>
    <vertices>
     <vertex x="0" y="0" z="0"/><vertex x="1" y="0" z="0"/>
     <vertex x="0" y="1" z="0"/><vertex x="0" y="0" z="1"/>
    </vertices>
    <triangles>
     <triangle v1="0" v2="2" v3="1"/><triangle v1="0" v2="1" v3="3"/>
     <triangle v1="1" v2="2" v3="3"/><triangle v1="2" v2="0" v3="3"/>
    </triangles>
   </mesh>
  </object>
 </resources>
 <build><item objectid="7" transform="1 0 0 0 1 0 0 0 1 1 0 0"/></build>
</model>"##;

    fn package(model: &str) -> Vec<u8> {
        let mut zip = ZipWriter::new(Cursor::new(Vec::new()));
        let options = SimpleFileOptions::default();
        zip.start_file(MODEL_PATH, options).unwrap();
        zip.write_all(model.as_bytes()).unwrap();
        zip.finish().unwrap().into_inner()
    }

    #[test]
    fn reads_units_transforms_and_colours_from_other_tools() {
        let meshes = read(&package(FOREIGN)).unwrap();
        assert_eq!(meshes.len(), 1);
        let m = &meshes[0];
        assert_eq!(m.name, "part");
        assert_eq!(m.color, Some([0, 0, 255, 255]));
        // Translated by 1 inch, then scaled to millimetres.
        assert!((m.positions[0][0] - 25.4).abs() < 1e-12);
        assert!((m.positions[1][0] - 50.8).abs() < 1e-12);
        assert!(m.is_watertight());
    }

    #[test]
    fn reader_rejects_components_and_bad_references() {
        let components = FOREIGN.replace(
            "<mesh>",
            "<components><component objectid=\"1\"/></components><mesh>",
        );
        assert!(read(&package(&components)).is_err());
        let missing = FOREIGN.replace(r#"objectid="7""#, r#"objectid="9""#);
        assert!(read(&package(&missing)).is_err());
        let bad_index = FOREIGN.replace(
            r#"v3="3"/><triangle v1="2""#,
            r#"v3="30"/><triangle v1="2""#,
        );
        assert!(read(&package(&bad_index)).is_err());
        let bad_unit = FOREIGN.replace("inch", "furlong");
        assert!(read(&package(&bad_unit)).is_err());
    }
}
