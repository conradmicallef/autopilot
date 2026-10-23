// Previous vs current tuning on the hold-out flights (240, seed 777): passes, field landings, and how many
// arrivals were hard (> 4 ft/s) or crash-grade (> 7 ft/s), split nominal / perturbed airframe.
use circuitsim::autotune::{cases, fly};
use circuitsim::tuning::Tuning;
use circuitsim::units::FT;
use circuitsim::ModelKind;
fn main() {
    let prev = Tuning {
        k_theta: 3.52, k_q: 2.94, ki_theta: 1.28, k_phi: 3.25, k_p: 0.53, k_beta: 2.35, k_r: 1.07,
        flare_lead: 2.5, flare_lead_short: 2.11, flare_lead_eo: 2.56, vapp_eo_kt: 67.18, gust_k: 1.33, gust_k_eo: 0.87, lull_vs: 1.29,
        eo_margin_factor: 1.1, eo_margin_height: 20.0, vbg_kt: 70.0, key_margin: 150.0, eo_turn_bank: 45.0, stretch_margin: 10.0,
    };
    let test = cases(240, 777.0);
    for (name, tu) in [("previous", prev), ("tuned", Tuning::default())] {
        let (mut pass, mut field, mut hard, mut crash, mut pn, mut pp) = (0, 0, 0, 0, 0, 0);
        for (i, c) in test.iter().enumerate() {
            let s = fly(c, ModelKind::SixDof, &tu);
            if s.passed() { pass += 1; if i % 2 == 0 { pn += 1 } else { pp += 1 } }
            if let Some(td) = &s.touchdown {
                if !td.on_runway { field += 1; }
                if td.sink / FT > 4.0 { hard += 1; if std::env::var("WHERE").is_ok() { eprintln!("  {name} hard {:.1} ft/s on_runway {} case {:?}", td.sink / FT, td.on_runway, c.kind); } }
                if td.sink / FT > 7.0 { crash += 1; }
            }
        }
        println!("{name:<9}: {pass}/240 passed (nominal {pn}/120, perturbed {pp}/120); off-runway touchdowns {field}; hard >4 ft/s {hard}; crash-grade >7 ft/s {crash}");
    }
}
