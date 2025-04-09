// middleware/errorHandler.js
exports.errorHandler = (err, req, res, next) => {
    console.error(err);
    res.status(500).json({
        success: false,
        message: err.message || 'Internal Server Error'
    });
};
