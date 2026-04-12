//! Constraint icons, text descriptions, viewport markers, and element-ref mapping.

use brep_sketch::SketchConstraint;

use crate::editor::CommittedCrossConstraint;
use crate::icons::{
    icon_angle, icon_coincident, icon_equal_len, icon_horizontal, icon_length,
    icon_parallel, icon_perp, icon_vertical,
};

// ── Icon selectors ────────────────────────────────────────────────────────────

pub(super) fn constraint_icon(c: &SketchConstraint) -> egui::ImageSource<'static> {
    match c {
        SketchConstraint::Parallel { .. }      => icon_parallel(),
        SketchConstraint::Perpendicular { .. }  => icon_perp(),
        SketchConstraint::Angle { .. }          => icon_angle(),
        SketchConstraint::Horizontal { .. }     => icon_horizontal(),
        SketchConstraint::Vertical { .. }       => icon_vertical(),
        SketchConstraint::EqualLength { .. }    => icon_equal_len(),
        SketchConstraint::Coincident { .. }     => icon_coincident(),
        SketchConstraint::FixedLength { .. }    => icon_length(),
        SketchConstraint::PointDistance { .. }  => icon_length(),
        SketchConstraint::PointFixed { .. }     => icon_coincident(),
        SketchConstraint::PointOnOrigin { .. }  => icon_coincident(),
        SketchConstraint::PointOnXAxis { .. }   => icon_horizontal(),
        SketchConstraint::PointOnYAxis { .. }   => icon_vertical(),
        SketchConstraint::HorizontalPair { .. } => icon_horizontal(),
        SketchConstraint::VerticalPair { .. }   => icon_vertical(),
        SketchConstraint::PointOnLine { .. }    => icon_coincident(),
        SketchConstraint::PointOnCircle { .. }  => icon_coincident(),
    }
}

pub(super) fn cross_constraint_icon(cc: &CommittedCrossConstraint) -> egui::ImageSource<'static> {
    match cc {
        CommittedCrossConstraint::Parallel { .. }       => icon_parallel(),
        CommittedCrossConstraint::Perpendicular { .. }  => icon_perp(),
        CommittedCrossConstraint::EqualLength { .. }    => icon_equal_len(),
        CommittedCrossConstraint::Angle { .. }          => icon_angle(),
        CommittedCrossConstraint::HorizontalPair { .. } => icon_horizontal(),
        CommittedCrossConstraint::VerticalPair { .. }   => icon_vertical(),
        CommittedCrossConstraint::Symmetric { .. }
        | CommittedCrossConstraint::SymmetricPoints { .. } => icon_equal_len(),
    }
}

// ── Text descriptions ─────────────────────────────────────────────────────────

/// Short text description of a constraint (used alongside its icon in the constraint list).
pub(crate) fn constraint_text(c: &SketchConstraint) -> String {
    match c {
        SketchConstraint::Parallel { seg_a, seg_b } =>
            format!("Parallel  {seg_a} ∥ {seg_b}"),
        SketchConstraint::Perpendicular { seg_a, seg_b } =>
            format!("Perpendicular  {seg_a} ⊥ {seg_b}"),
        SketchConstraint::Angle { seg_a, seg_b, degrees } =>
            format!("Angle  {degrees:.0}°  ({seg_a}/{seg_b})"),
        SketchConstraint::Horizontal { seg } =>
            format!("Horizontal  seg {seg}"),
        SketchConstraint::Vertical { seg } =>
            format!("Vertical  seg {seg}"),
        SketchConstraint::EqualLength { seg_a, seg_b } =>
            format!("Equal Length  {seg_a} = {seg_b}"),
        SketchConstraint::Coincident { pt_a, pt_b } =>
            format!("Coincident  pt {pt_a} = {pt_b}"),
        SketchConstraint::FixedLength { seg, value } =>
            format!("Length  seg {seg} = {value:.3}"),
        SketchConstraint::PointDistance { pt_a, pt_b, value } =>
            format!("Distance  pt {pt_a}–{pt_b} = {value:.3}"),
        SketchConstraint::PointFixed { .. }       => String::new(),
        SketchConstraint::PointOnOrigin { pt }    => format!("On Origin  pt {pt}"),
        SketchConstraint::PointOnXAxis { pt }     => format!("On X-axis  pt {pt}"),
        SketchConstraint::PointOnYAxis { pt }     => format!("On Y-axis  pt {pt}"),
        SketchConstraint::HorizontalPair { pt_a, pt_b, .. } => format!("Horizontal  pt {pt_a}–{pt_b}"),
        SketchConstraint::VerticalPair   { pt_a, pt_b, .. } => format!("Vertical  pt {pt_a}–{pt_b}"),
        SketchConstraint::PointOnLine { pt, seg }           => format!("Coincident  pt {pt} on seg {seg}"),
        SketchConstraint::PointOnCircle { pt, .. }          => format!("Coincident  pt {pt} on curve"),
    }
}

/// Returns (vertex refs: Vec<(pi, vi)>, segment refs: Vec<(pi, si)>) referenced by a cross-constraint.
pub(super) fn cross_constraint_element_refs(cc: &CommittedCrossConstraint)
    -> (Vec<(usize, usize)>, Vec<(usize, usize)>)
{
    match cc {
        CommittedCrossConstraint::Parallel      { pi_a, si_a, pi_b, si_b }
        | CommittedCrossConstraint::Perpendicular { pi_a, si_a, pi_b, si_b }
        | CommittedCrossConstraint::EqualLength   { pi_a, si_a, pi_b, si_b } =>
            (vec![], vec![(*pi_a, *si_a), (*pi_b, *si_b)]),
        CommittedCrossConstraint::Angle { pi_a, si_a, pi_b, si_b, .. } =>
            (vec![], vec![(*pi_a, *si_a), (*pi_b, *si_b)]),
        CommittedCrossConstraint::HorizontalPair { pi_a, vi_a, pi_b, vi_b, .. }
        | CommittedCrossConstraint::VerticalPair  { pi_a, vi_a, pi_b, vi_b, .. } =>
            (vec![(*pi_a, *vi_a), (*pi_b, *vi_b)], vec![]),
        CommittedCrossConstraint::Symmetric { pi_a, si_a, pi_b, si_b, .. } =>
            (vec![], vec![(*pi_a, *si_a), (*pi_b, *si_b)]),
        CommittedCrossConstraint::SymmetricPoints { pi_a, vi_a, pi_b, vi_b, pi_c, vi_c } =>
            (vec![(*pi_a, *vi_a), (*pi_b, *vi_b), (*pi_c, *vi_c)], vec![]),
    }
}

pub(super) fn cross_constraint_text(cc: &CommittedCrossConstraint) -> String {
    match cc {
        CommittedCrossConstraint::Parallel { pi_a, si_a, pi_b, si_b } =>
            format!("Parallel  P{pi_a}:{si_a} ∥ P{pi_b}:{si_b}"),
        CommittedCrossConstraint::Perpendicular { pi_a, si_a, pi_b, si_b } =>
            format!("Perpendicular  P{pi_a}:{si_a} ⊥ P{pi_b}:{si_b}"),
        CommittedCrossConstraint::EqualLength { pi_a, si_a, pi_b, si_b } =>
            format!("Equal Length  P{pi_a}:{si_a} = P{pi_b}:{si_b}"),
        CommittedCrossConstraint::Angle { pi_a, si_a, pi_b, si_b, degrees } =>
            format!("Angle  {degrees:.0}°  (P{pi_a}:{si_a}/P{pi_b}:{si_b})"),
        CommittedCrossConstraint::HorizontalPair { pi_a, vi_a, pi_b, vi_b, .. } =>
            format!("Horizontal  P{pi_a}:{vi_a} — P{pi_b}:{vi_b}"),
        CommittedCrossConstraint::VerticalPair { pi_a, vi_a, pi_b, vi_b, .. } =>
            format!("Vertical  P{pi_a}:{vi_a} — P{pi_b}:{vi_b}"),
        CommittedCrossConstraint::Symmetric { pi_a, si_a, pi_b, si_b, .. } =>
            format!("Symmetric  P{pi_a}:{si_a} / P{pi_b}:{si_b}"),
        CommittedCrossConstraint::SymmetricPoints { pi_a, vi_a, pi_b, vi_b, pi_c, vi_c } =>
            format!("Symmetric  P{pi_a}:{vi_a} / P{pi_b}:{vi_b} ↔ P{pi_c}:{vi_c}"),
    }
}

// ── Element-ref mapping ───────────────────────────────────────────────────────

/// Returns the (point indices, segment indices) referenced by a constraint.
pub(super) fn constraint_element_refs(c: &SketchConstraint) -> (Vec<usize>, Vec<usize>) {
    match c {
        SketchConstraint::Horizontal { seg }             => (vec![], vec![*seg]),
        SketchConstraint::Vertical { seg }               => (vec![], vec![*seg]),
        SketchConstraint::FixedLength { seg, .. }        => (vec![], vec![*seg]),
        SketchConstraint::Parallel { seg_a, seg_b }      => (vec![], vec![*seg_a, *seg_b]),
        SketchConstraint::Perpendicular { seg_a, seg_b } => (vec![], vec![*seg_a, *seg_b]),
        SketchConstraint::EqualLength { seg_a, seg_b }   => (vec![], vec![*seg_a, *seg_b]),
        SketchConstraint::Angle { seg_a, seg_b, .. }     => (vec![], vec![*seg_a, *seg_b]),
        SketchConstraint::PointOnOrigin { pt }           => (vec![*pt], vec![]),
        SketchConstraint::PointOnXAxis { pt }            => (vec![*pt], vec![]),
        SketchConstraint::PointOnYAxis { pt }            => (vec![*pt], vec![]),
        SketchConstraint::PointFixed { pt, .. }          => (vec![*pt], vec![]),
        SketchConstraint::Coincident { pt_a, pt_b }      => (vec![*pt_a, *pt_b], vec![]),
        SketchConstraint::PointOnLine { pt, seg }        => (vec![*pt], vec![*seg]),
        SketchConstraint::PointDistance { pt_a, pt_b, .. } => (vec![*pt_a, *pt_b], vec![]),
        SketchConstraint::PointOnCircle { pt, .. }       => (vec![*pt], vec![]),
        SketchConstraint::HorizontalPair { pt_a, pt_b, .. } => (vec![*pt_a, *pt_b], vec![]),
        SketchConstraint::VerticalPair { pt_a, pt_b, .. }   => (vec![*pt_a, *pt_b], vec![]),
    }
}

// ── Viewport marker drawing ───────────────────────────────────────────────────

/// Draw a small symbol near the segment midpoint to indicate a constraint.
pub(super) fn draw_constraint_marker(
    painter: &egui::Painter,
    c: &SketchConstraint,
    points: &[brep_core::Point3],
    n: usize,
    proj: &impl Fn(brep_core::Point3) -> Option<egui::Pos2>,
    selected: bool,
    hovered: bool,
) {
    let midpoint = |seg: usize| -> Option<egui::Pos2> {
        let a = proj(points[seg])?;
        let b = proj(points[(seg + 1) % n])?;
        Some(egui::pos2((a.x + b.x) * 0.5, (a.y + b.y) * 0.5))
    };
    let color = if selected {
        egui::Color32::from_rgb(255, 160, 60)   // orange — matches element selection
    } else if hovered {
        egui::Color32::from_rgb(220, 210, 100)  // bright yellow-gold on hover
    } else {
        egui::Color32::from_rgb(160, 155, 55)   // subdued default
    };
    let font = egui::FontId::proportional(10.0);
    let draw = |seg: usize, sym: &str| {
        if let Some(m) = midpoint(seg) {
            painter.text(m + egui::vec2(6.0, -6.0), egui::Align2::LEFT_BOTTOM, sym, font.clone(), color);
        }
    };
    match c {
        SketchConstraint::Parallel { seg_a, seg_b } => {
            draw(*seg_a, "∥"); draw(*seg_b, "∥");
        }
        SketchConstraint::Perpendicular { seg_a, seg_b } => {
            draw(*seg_a, "⊥"); draw(*seg_b, "⊥");
        }
        SketchConstraint::Angle { seg_a, degrees, .. } => {
            draw(*seg_a, &format!("∠{:.0}°", degrees));
        }
        SketchConstraint::Horizontal { seg } => { draw(*seg, "↔"); }
        SketchConstraint::Vertical { seg } => { draw(*seg, "↕"); }
        SketchConstraint::EqualLength { seg_a, seg_b } => {
            draw(*seg_a, "≡"); draw(*seg_b, "≡");
        }
        SketchConstraint::Coincident { pt_a, .. } => {
            if let Some(p) = proj(points[*pt_a]) {
                painter.circle_stroke(p, 5.0, egui::Stroke::new(1.2, color));
            }
        }
        SketchConstraint::FixedLength { seg, value } => {
            draw(*seg, &format!("⟺{value:.2}"));
        }
        SketchConstraint::PointDistance { pt_a, pt_b, value } => {
            if let (Some(a), Some(b)) = (proj(points[*pt_a]), proj(points[*pt_b])) {
                let mid = egui::pos2((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
                painter.text(
                    mid + egui::vec2(6.0, -6.0),
                    egui::Align2::LEFT_BOTTOM,
                    &format!("⟺{value:.2}"),
                    font.clone(),
                    color,
                );
            }
        }
        SketchConstraint::PointFixed { .. } => {} // internal drag pin — no overlay marker
        SketchConstraint::PointOnOrigin { pt } => {
            if let Some(p) = proj(points[*pt]) {
                painter.text(p + egui::vec2(6.0, -6.0), egui::Align2::LEFT_BOTTOM, "⊙", font.clone(), color);
            }
        }
        SketchConstraint::PointOnXAxis { pt } => {
            if let Some(p) = proj(points[*pt]) {
                painter.text(p + egui::vec2(6.0, -6.0), egui::Align2::LEFT_BOTTOM, "∈X", font.clone(), color);
            }
        }
        SketchConstraint::PointOnYAxis { pt } => {
            if let Some(p) = proj(points[*pt]) {
                painter.text(p + egui::vec2(6.0, -6.0), egui::Align2::LEFT_BOTTOM, "∈Y", font.clone(), color);
            }
        }
        SketchConstraint::HorizontalPair { pt_a, pt_b, .. } => {
            if let (Some(a), Some(b)) = (proj(points[*pt_a]), proj(points[*pt_b])) {
                let mid = egui::pos2((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
                painter.text(mid + egui::vec2(6.0, -6.0), egui::Align2::LEFT_BOTTOM, "↔", font.clone(), color);
            }
        }
        SketchConstraint::VerticalPair { pt_a, pt_b, .. } => {
            if let (Some(a), Some(b)) = (proj(points[*pt_a]), proj(points[*pt_b])) {
                let mid = egui::pos2((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
                painter.text(mid + egui::vec2(6.0, -6.0), egui::Align2::LEFT_BOTTOM, "↕", font.clone(), color);
            }
        }
        SketchConstraint::PointOnLine { pt, seg } => {
            if let Some(p) = proj(points[*pt]) {
                painter.circle_stroke(p, 5.0, egui::Stroke::new(1.2, color));
            }
            if let (Some(a), Some(b)) = (proj(points[*seg]), proj(points[(*seg + 1) % n])) {
                let m = egui::pos2((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
                painter.text(m + egui::vec2(6.0, -6.0), egui::Align2::LEFT_BOTTOM, "◯", font.clone(), color);
            }
        }
        SketchConstraint::PointOnCircle { pt, .. } => {
            if let Some(p) = proj(points[*pt]) {
                painter.circle_stroke(p, 5.0, egui::Stroke::new(1.2, color));
                painter.text(p + egui::vec2(6.0, -6.0), egui::Align2::LEFT_BOTTOM, "◎", font.clone(), color);
            }
        }
    }
}

// ── Viewport panel snap ───────────────────────────────────────────────────────

/// Snap a floating panel's top-left position to the nearest viewport edge if within SNAP_DIST.
pub(crate) fn snap_to_viewport(pos: egui::Pos2, size: egui::Vec2, viewport: egui::Rect) -> egui::Pos2 {
    const GAP: f32 = 12.0;
    const SNAP: f32 = 40.0;
    let mut p = pos;
    // Clamp inside viewport first.
    p.x = p.x.clamp(viewport.left() + GAP, (viewport.right()  - GAP - size.x).max(viewport.left() + GAP));
    p.y = p.y.clamp(viewport.top()  + GAP, (viewport.bottom() - GAP - size.y).max(viewport.top()  + GAP));
    // Then snap to edges.
    if      (p.x            - (viewport.left()   + GAP)).abs() < SNAP { p.x = viewport.left()   + GAP; }
    else if (p.x + size.x   - (viewport.right()  - GAP)).abs() < SNAP { p.x = viewport.right()  - GAP - size.x; }
    if      (p.y            - (viewport.top()    + GAP)).abs() < SNAP { p.y = viewport.top()    + GAP; }
    else if (p.y + size.y   - (viewport.bottom() - GAP)).abs() < SNAP { p.y = viewport.bottom() - GAP - size.y; }
    p
}
