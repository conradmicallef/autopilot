//! Minimal JSON writer for the browser frame. Numbers are written with fixed precision using integer
//! arithmetic, so the WASM build carries neither serde_json nor the general float formatter.

pub struct Json {
    pub s: String,
    first: Vec<bool>,
}

impl Default for Json {
    fn default() -> Self {
        Json { s: String::with_capacity(4096), first: Vec::new() }
    }
}

impl Json {
    fn sep(&mut self) {
        if let Some(f) = self.first.last_mut() {
            if *f {
                *f = false;
            } else {
                self.s.push(',');
            }
        }
    }
    pub fn obj(&mut self) -> &mut Self {
        self.sep();
        self.s.push('{');
        self.first.push(true);
        self
    }
    pub fn arr(&mut self) -> &mut Self {
        self.sep();
        self.s.push('[');
        self.first.push(true);
        self
    }
    pub fn end_obj(&mut self) -> &mut Self {
        self.first.pop();
        self.s.push('}');
        self
    }
    pub fn end_arr(&mut self) -> &mut Self {
        self.first.pop();
        self.s.push(']');
        self
    }
    /// Object key; the next value call writes its value.
    pub fn k(&mut self, key: &str) -> &mut Self {
        self.sep();
        self.str_raw(key);
        self.s.push(':');
        if let Some(f) = self.first.last_mut() {
            *f = true; // value follows without a comma
        }
        self
    }
    fn str_raw(&mut self, v: &str) {
        self.s.push('"');
        for c in v.chars() {
            match c {
                '"' => self.s.push_str("\\\""),
                '\\' => self.s.push_str("\\\\"),
                '\n' => self.s.push_str("\\n"),
                c if (c as u32) < 0x20 => self.s.push(' '),
                c => self.s.push(c),
            }
        }
        self.s.push('"');
    }
    pub fn str(&mut self, v: &str) -> &mut Self {
        self.sep();
        self.str_raw(v);
        self.after_value();
        self
    }
    /// Number with up to 5 decimals (trailing zeros trimmed); non-finite → null.
    pub fn num(&mut self, v: f64) -> &mut Self {
        self.sep();
        if !v.is_finite() {
            self.s.push_str("null");
        } else if v.abs() >= 9.0e12 {
            self.int_raw(v.round() as i64);
        } else {
            let scaled = (v * 100_000.0).round() as i64;
            if scaled < 0 {
                self.s.push('-');
            }
            let a = scaled.unsigned_abs();
            self.uint_raw(a / 100_000);
            let mut frac = a % 100_000;
            if frac != 0 {
                self.s.push('.');
                let mut digits = 5;
                while frac % 10 == 0 {
                    frac /= 10;
                    digits -= 1;
                }
                let txt = frac.to_string();
                for _ in txt.len()..digits {
                    self.s.push('0');
                }
                self.s.push_str(&txt);
            }
        }
        self.after_value();
        self
    }
    fn uint_raw(&mut self, v: u64) {
        self.s.push_str(&v.to_string());
    }
    fn int_raw(&mut self, v: i64) {
        self.s.push_str(&v.to_string());
    }
    pub fn int(&mut self, v: i64) -> &mut Self {
        self.sep();
        self.int_raw(v);
        self.after_value();
        self
    }
    pub fn bool(&mut self, v: bool) -> &mut Self {
        self.sep();
        self.s.push_str(if v { "true" } else { "false" });
        self.after_value();
        self
    }
    pub fn null(&mut self) -> &mut Self {
        self.sep();
        self.s.push_str("null");
        self.after_value();
        self
    }
    pub fn opt_num(&mut self, v: Option<f64>) -> &mut Self {
        match v {
            Some(x) => self.num(x),
            None => self.null(),
        }
    }
    pub fn opt_bool(&mut self, v: Option<bool>) -> &mut Self {
        match v {
            Some(x) => self.bool(x),
            None => self.null(),
        }
    }
    pub fn opt_str(&mut self, v: Option<&str>) -> &mut Self {
        match v {
            Some(x) => self.str(x),
            None => self.null(),
        }
    }
    pub fn point(&mut self, p: [f64; 2]) -> &mut Self {
        self.arr().num(p[0]).num(p[1]).end_arr()
    }
    // values written directly after a key must not reset the separator state of the enclosing object
    fn after_value(&mut self) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn writes_valid_json() {
        let mut j = Json::default();
        j.obj().k("a").num(1.5).k("b").arr().num(-0.000004).num(2.0).num(-3.25).end_arr().k("c").str("x\"y").k("d").null().end_obj();
        assert_eq!(j.s, r#"{"a":1.5,"b":[0,2,-3.25],"c":"x\"y","d":null}"#);
    }
}
