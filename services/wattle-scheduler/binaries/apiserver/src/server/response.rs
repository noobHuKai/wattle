use axum::{
    Json,
    http::StatusCode,
    response::{IntoResponse, Response},
};
use serde::Serialize;

/// 统一的 API 响应结构
#[derive(Debug, Clone, Serialize)]
pub struct ApiResponse<T> {
    pub code: u32,
    pub data: Option<T>,
    pub msg: Option<String>,
}

impl<T> ApiResponse<T> {
    pub fn success(data: T) -> Self {
        ApiResponse {
            code: 200,
            data: Some(data),
            msg: None,
        }
    }

    pub fn error(msg: String) -> Self {
        ApiResponse {
            code: 500,
            data: None,
            msg: Some(msg),
        }
    }
}

/// 应用错误类型
pub struct AppError(pub eyre::Report);

impl<E> From<E> for AppError
where
    E: Into<eyre::Report>,
{
    fn from(err: E) -> Self {
        Self(err.into())
    }
}

impl IntoResponse for AppError {
    fn into_response(self) -> Response {
        let err = self.0;
        tracing::error!("API Error: {err:?}");

        let api_response: ApiResponse<()> = ApiResponse::error(err.to_string());
        (StatusCode::OK, Json(api_response)).into_response()
    }
}

pub type Result<T, E = AppError> = std::result::Result<T, E>;
pub type ApiJsonResult<T> = Result<Json<ApiResponse<T>>>;

/// 帮助 trait 用于将结果转换为 JSON 响应
pub trait IntoApiJsonResult<T> {
    fn into_success_json(self) -> ApiJsonResult<T>;
}

impl<T> IntoApiJsonResult<T> for T {
    fn into_success_json(self) -> ApiJsonResult<T> {
        Ok(Json(ApiResponse::success(self)))
    }
}

impl<T> IntoApiJsonResult<T> for Result<T> {
    fn into_success_json(self) -> ApiJsonResult<T> {
        match self {
            Ok(data) => Ok(Json(ApiResponse::success(data))),
            Err(err) => Err(err),
        }
    }
}
