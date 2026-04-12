//! Modal dialogs: length input and angle input.

use brep_sketch::SketchConstraint;

use crate::editor::{
    AngleDialogTarget, CommittedCrossConstraint, EditorState, LengthTarget, UiAction,
};

// ── Length modal ──────────────────────────────────────────────────────────────

pub(super) fn draw_length_modal(
    ctx: &egui::Context,
    editor: &EditorState,
    target: LengthTarget,
    initial_len: f64,
) -> Vec<UiAction> {
    let mut actions: Vec<UiAction> = Vec::new();

    let len_id      = egui::Id::new("sketch_length_value");
    let len_init_id = egui::Id::new("sketch_length_initialized");
    let description = match target {
        LengthTarget::Segment(seg)          => format!("Length of segment {seg}:"),
        LengthTarget::Points(a, b)          => format!("Distance between vertex {a} and vertex {b}:"),
        LengthTarget::CommittedSegment(pi, si) => format!("Length of segment {si} on profile {pi}:"),
        LengthTarget::CommittedRadius(pi)   => format!("Radius of profile {pi}:"),
    };

    // Seed the input with the actual current length the first time this dialog opens.
    let already_initialized: bool = ctx.data(|d| d.get_temp(len_init_id).unwrap_or(false));
    if !already_initialized {
        ctx.data_mut(|d| {
            d.insert_temp(len_id, initial_len);
            d.insert_temp(len_init_id, true);
        });
    }

    let modal = egui::Modal::new(egui::Id::new("length_input_modal")).show(ctx, |ui| {
        ui.set_min_width(240.0);
        ui.heading("Set Length");
        ui.add_space(4.0);
        ui.label(&description);
        ui.add_space(6.0);

        let mut val: f64 = ctx.data_mut(|d| *d.get_temp_mut_or(len_id, initial_len));
        let resp = ui.add(
            egui::DragValue::new(&mut val)
                .speed(0.01)
                .range(1e-3..=1e6)
                .min_decimals(3),
        );
        if resp.changed() {
            ctx.data_mut(|d| d.insert_temp(len_id, val));
        }

        ui.add_space(10.0);
        ui.horizontal(|ui| {
            let apply  = ui.button("Apply");
            let cancel = ui.button("Cancel");

            let confirm = apply.clicked()  || ui.input(|i| i.key_pressed(egui::Key::Enter));
            let dismiss = cancel.clicked() || ui.input(|i| i.key_pressed(egui::Key::Escape));

            if confirm {
                let value = ctx.data(|d| d.get_temp(len_id).unwrap_or(initial_len));
                ctx.data_mut(|d| d.remove::<bool>(len_init_id));
                match target {
                    LengthTarget::Segment(_) => {
                        // Apply to all currently selected segments (multi-select support).
                        let segs: Vec<usize> = editor.sketch.as_ref()
                            .map(|sk| sk.sel_segs().collect())
                            .unwrap_or_default();
                        let segs = if segs.is_empty() {
                            match target { LengthTarget::Segment(s) => vec![s], _ => vec![] }
                        } else { segs };
                        for seg in segs {
                            actions.push(UiAction::SketchAddConstraint(
                                SketchConstraint::FixedLength { seg, value }
                            ));
                        }
                    }
                    LengthTarget::Points(pt_a, pt_b) => {
                        actions.push(UiAction::SketchAddConstraint(
                            SketchConstraint::PointDistance { pt_a, pt_b, value }
                        ));
                    }
                    LengthTarget::CommittedSegment(pi, si) => {
                        actions.push(UiAction::SketchAddCommittedConstraint(
                            pi, SketchConstraint::FixedLength { seg: si, value }
                        ));
                    }
                    LengthTarget::CommittedRadius(pi) => {
                        if let Some(sk) = &editor.sketch {
                            if let Some(cp) = sk.committed_profiles.get(pi) {
                                use crate::editor::ProfileShape;
                                match &cp.shape {
                                    ProfileShape::Circle => {
                                        actions.push(UiAction::SketchAddCommittedConstraint(
                                            pi, SketchConstraint::PointDistance { pt_a: 0, pt_b: 1, value }
                                        ));
                                    }
                                    ProfileShape::Arc => {
                                        // Arc: points[0]=start, points[2]=center; radius = dist(start, center)
                                        actions.push(UiAction::SketchAddCommittedConstraint(
                                            pi, SketchConstraint::PointDistance { pt_a: 0, pt_b: 2, value }
                                        ));
                                    }
                                    _ => {}
                                }
                            }
                        }
                    }
                }
            } else if dismiss {
                ctx.data_mut(|d| d.remove::<bool>(len_init_id));
                actions.push(UiAction::SketchCancelLengthInput);
            }
        });
    });

    if modal.should_close() {
        ctx.data_mut(|d| d.remove::<bool>(len_init_id));
        actions.push(UiAction::SketchCancelLengthInput);
    }

    actions
}

// ── Angle modal ───────────────────────────────────────────────────────────────

pub(super) fn draw_angle_modal(
    ctx: &egui::Context,
    angle_target: AngleDialogTarget,
) -> Vec<UiAction> {
    let mut actions: Vec<UiAction> = Vec::new();

    let angle_id = egui::Id::new("sketch_angle_deg");

    let modal = egui::Modal::new(egui::Id::new("angle_input_modal")).show(ctx, |ui| {
        ui.set_min_width(220.0);
        ui.heading("Set Angle");
        ui.add_space(4.0);
        let desc = match angle_target {
            AngleDialogTarget::ActiveProfile { seg_a, seg_b } =>
                format!("Angle between segment {seg_a} and segment {seg_b}:"),
            AngleDialogTarget::CrossProfile { pi_a, si_a, pi_b, si_b } =>
                format!("Angle between profile {pi_a} seg {si_a} and profile {pi_b} seg {si_b}:"),
            AngleDialogTarget::CommittedProfile { pi, si_a, si_b } =>
                format!("Angle between segment {si_a} and segment {si_b} on profile {pi}:"),
        };
        ui.label(desc);
        ui.add_space(6.0);

        let mut deg: f64 = ctx.data_mut(|d| *d.get_temp_mut_or(angle_id, 90.0_f64));
        let resp = ui.add(
            egui::DragValue::new(&mut deg)
                .speed(1.0)
                .range(1.0..=179.0)
                .suffix(" °"),
        );
        if resp.changed() {
            ctx.data_mut(|d| *d.get_temp_mut_or::<f64>(angle_id, 90.0) = deg);
        }

        ui.add_space(10.0);
        ui.horizontal(|ui| {
            let apply  = ui.button("Apply");
            let cancel = ui.button("Cancel");

            if apply.clicked() || ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                let degrees = ctx.data_mut(|d| *d.get_temp_mut_or::<f64>(angle_id, 90.0));
                match angle_target {
                    AngleDialogTarget::ActiveProfile { seg_a, seg_b } => {
                        actions.push(UiAction::SketchAddConstraint(SketchConstraint::Angle {
                            seg_a, seg_b, degrees,
                        }));
                    }
                    AngleDialogTarget::CrossProfile { pi_a, si_a, pi_b, si_b } => {
                        actions.push(UiAction::SketchAddCrossConstraint(
                            CommittedCrossConstraint::Angle { pi_a, si_a, pi_b, si_b, degrees }
                        ));
                    }
                    AngleDialogTarget::CommittedProfile { pi, si_a, si_b } => {
                        actions.push(UiAction::SketchAddCommittedConstraint(
                            pi, SketchConstraint::Angle { seg_a: si_a, seg_b: si_b, degrees }
                        ));
                    }
                }
            }
            if cancel.clicked() || ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                actions.push(UiAction::SketchCancelAngleInput);
            }
        });
    });

    if modal.should_close() {
        actions.push(UiAction::SketchCancelAngleInput);
    }

    actions
}
