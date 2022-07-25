from flask import Flask, request
from twilio.twiml.voice_response import VoiceResponse

app = Flask(__name__)


@app.route("/answer", methods=['GET', 'POST'])
def voice():
    """Respond to incoming phone calls"""
    
    # Start our TwiML response
    resp = VoiceResponse()
    
    # Play an audio file for the caller
    resp.play('https://www.mboxdrive.com/kontol.mp3')

    return str(resp)

if __name__ == "__main__":
    app.run(debug=True)