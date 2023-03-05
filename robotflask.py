from flask import Flask, render_template, Response,jsonify


app = Flask(__name__)
app.debug=True


class JSONResponse(Response):
    default_mimetype = 'application/json'

    @classmethod
    def force_type(cls, response, environ=None):
        if isinstance(response, dict):
            response = jsonify(response)
        return super(JSONResponse, cls).force_type(response, environ)


app.response_class = JSONResponse

@app.route('/detection/',methods=['GET','POST'])
def detection():
    return {"detection": "True"}

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=9999,threaded=True)


    # app = QtWidgets.QApplication(sys.argv)
    #
    # splash = QtWidgets.QSplashScreen(
    #     QtGui.QPixmap(os.path.join("Resources", "images", "splash")))
    # splash.show()
    #
    # main = PVision()
    #
    # splash.finish(main)
    # main.show()
    # sys.exit(app.exec_())