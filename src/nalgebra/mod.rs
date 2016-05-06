use openvr::*;

extern crate nalgebra;

pub trait ToMatrix<MatType> {
    /// converts to a matrix
    fn to_matrix(&self) -> MatType;
}

/// Allows to convert a tracked device pose to an transformation matrix
impl ToMatrix<nalgebra::Matrix4> for TrackedDevicePose {
    pub fn to_matrix(&self) -> MatType {
        let raw = self.to_device;
        nalgebra::Matrix4::new(
            raw[0][0], raw[0][1], raw[0][2], raw[0][3],
            raw[1][0], raw[1][1], raw[1][2], raw[1][3],
            raw[2][0], raw[2][1], raw[2][2], raw[2][3],
            0.0, 0.0, 0.0, 1.0)
    }
}

/// Gets the projection matrix from openvr and converts it to an nalgebra matrix
pub fn projection_matrix(system: &IVRSystem, eye: Eye, near: f32, far: f32) -> nalgebra::Matrix4 {
    let raw = system.projection_matrix(eye, near, far);
    nalgebra::Matrix4::new(
        raw[0][0], raw[0][1], raw[0][2], raw[0][3],
        raw[1][0], raw[1][1], raw[1][2], raw[1][3],
        raw[2][0], raw[2][1], raw[2][2], raw[2][3],
        raw[3][0], raw[3][1], raw[3][2], raw[3][3])
}

/// Gets the eye to head matrix from openvr and converts it to an nalgebra transformation matrix
pub fn eye_to_head_matrix(system: &IVRSystem, eye: Eye) -> nalgebra::Matrix4 {
    let raw = system.eye_to_head_transform(eye);
    nalgebra::Matrix4::new(
        raw[0][0], raw[0][1], raw[0][2], raw[0][3],
        raw[1][0], raw[1][1], raw[1][2], raw[1][3],
        raw[2][0], raw[2][1], raw[2][2], raw[2][3],
        0.0, 0.0, 0.0, 1.0)
}
